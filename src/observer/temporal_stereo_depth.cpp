#include "mec/observer/temporal_stereo_depth.h"

#include <algorithm>
#include <cmath>

namespace mec {

TemporalStereoDepth::TemporalStereoDepth(int max_w, int max_h)
    : TemporalStereoDepth(max_w, max_h, Config{}) {}

TemporalStereoDepth::TemporalStereoDepth(int max_w, int max_h, Config cfg)
    : lk_(max_w, max_h), cfg_(cfg) {}

float TemporalStereoDepth::kalman_fuse(float prior, float meas,
                                       float sigma_prior, float sigma_meas) {
    const float vp = sigma_prior * sigma_prior;
    const float vm = sigma_meas * sigma_meas;
    return (prior / vp + meas / vm) / (1.0f / vp + 1.0f / vm);
}

void TemporalStereoDepth::resolve(PoseObservation& obs, const Frame& prev,
                                  const Frame& curr, const IMUFrame& imu,
                                  const CameraIntrinsics& intr, float lens_prior_m) {
    const int n = std::min<int>(obs.keypoint_count, 33);
    if (n <= 0) return;

    const float b = imu.baseline_m;

    // Phone moving too fast: optical flow unreliable -> skip this frame
    // entirely, leaving the existing depth_hint estimates untouched.
    if (b > cfg_.max_baseline_m) return;

    // Phone effectively stationary: no stereo signal, use the lens prior.
    if (b < cfg_.min_baseline_m) {
        for (int i = 0; i < n; ++i) obs.keypoints[i].depth_hint = lens_prior_m;
        return;
    }

    const int w = curr.width, h = curr.height;
    for (int i = 0; i < n; ++i) {
        pts_[2 * i]     = obs.keypoints[i].x * static_cast<float>(w);
        pts_[2 * i + 1] = obs.keypoints[i].y * static_cast<float>(h);
    }

    // Flow of each keypoint from the current frame back to the previous one.
    lk_.track(curr.data, prev.data, w, h, pts_, n, flow_, status_);

    // Inter-frame camera-frame rotation (curr->prev directions): bring the
    // IMU's body-frame delta into the camera frame via the camera->body
    // extrinsic. dq_cam = R_cb^{-1} ⊗ dq_body ⊗ R_cb (§15.7).
    const Quat dq_body{imu.dqw, imu.dqx, imu.dqy, imu.dqz};
    const Quat dq_cam = mul(conj(cfg_.cam_to_body), mul(dq_body, cfg_.cam_to_body));

    const float fx = intr.fx, fy = intr.fy, cx = intr.cx, cy = intr.cy;

    // Translation direction in the camera frame (§15.7(a)). When known it
    // enables epipolar projection + subject-motion gating; when zero we fall
    // back to the plain flow-magnitude model (assumes lateral translation).
    const Vec3 t_cam = rotate(conj(cfg_.cam_to_body), Vec3{imu.tdx, imu.tdy, imu.tdz});
    const float t_norm = norm(t_cam);
    const bool have_dir = t_norm > 1e-3f;
    const Vec3 t_hat = have_dir ? t_cam * (1.0f / t_norm) : Vec3{};

    for (int i = 0; i < n; ++i) {
        if (!status_[i]) { obs.keypoints[i].depth_hint = lens_prior_m; continue; }

        // De-rotate (§15.7(b)): subtract the rotation-only displacement (the
        // infinite homography K·R·K^-1) so only translational flow remains.
        const float u = pts_[2 * i], v = pts_[2 * i + 1];
        const Vec3 ray{(u - cx) / fx, (v - cy) / fy, 1.0f};
        const Vec3 rp = rotate(dq_cam, ray);
        if (rp.z < 1e-3f) { obs.keypoints[i].depth_hint = lens_prior_m; continue; }
        const float fdx = flow_[2 * i]     - (fx * rp.x / rp.z + cx - u);
        const float fdy = flow_[2 * i + 1] - (fy * rp.y / rp.z + cy - v);

        if (!have_dir) {
            // Legacy model: |flow| is the disparity, lateral translation assumed.
            const float disparity = std::sqrt(fdx * fdx + fdy * fdy);
            if (disparity < cfg_.min_disparity_px) {
                obs.keypoints[i].depth_hint = lens_prior_m;
                continue;
            }
            obs.keypoints[i].depth_hint = kalman_fuse(
                lens_prior_m, fx * b / disparity, cfg_.sigma_lens, cfg_.sigma_imu);
            continue;
        }

        // Epipolar direction of the camera-translation motion field at this
        // pixel: d = (-fx*Tx + (u-cx)*Tz, -fy*Ty + (v-cy)*Tz). Its magnitude is
        // independent of depth; depth only scales the flow along it.
        const float dvx = -fx * t_hat.x + (u - cx) * t_hat.z;
        const float dvy = -fy * t_hat.y + (v - cy) * t_hat.z;
        const float dmag = std::sqrt(dvx * dvx + dvy * dvy);
        if (dmag < 1e-3f) { obs.keypoints[i].depth_hint = lens_prior_m; continue; } // at epipole
        const float dux = dvx / dmag, duy = dvy / dmag;

        // Split the de-rotated flow into epipolar (parallax) and perpendicular
        // (subject-motion) components.
        const float parallel = fdx * dux + fdy * duy;
        const float perpx = fdx - parallel * dux, perpy = fdy - parallel * duy;
        const float perp = std::sqrt(perpx * perpx + perpy * perpy);

        if (perp > cfg_.motion_gate_px) {
            obs.keypoints[i].depth_hint = lens_prior_m; // subject moving -> gate
            continue;
        }
        const float parallax = std::fabs(parallel);
        if (parallax < cfg_.min_disparity_px) {
            obs.keypoints[i].depth_hint = lens_prior_m; // pure rotation / too far
            continue;
        }
        // depth = baseline * |d| / parallax  (reduces to f*b/disparity laterally)
        obs.keypoints[i].depth_hint = kalman_fuse(
            lens_prior_m, b * dmag / parallax, cfg_.sigma_lens, cfg_.sigma_imu);
    }
}

} // namespace mec
