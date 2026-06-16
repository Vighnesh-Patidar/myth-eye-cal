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
    const float focal = fx; // assume ~square pixels (fx ~= fy)

    for (int i = 0; i < n; ++i) {
        if (!status_[i]) { obs.keypoints[i].depth_hint = lens_prior_m; continue; }

        // Predicted rotation-only location of this keypoint in the previous
        // frame: project the back-projected ray rotated by the inter-frame
        // rotation (the infinite homography K·R·K^-1).
        const float u = pts_[2 * i], v = pts_[2 * i + 1];
        const Vec3 ray{(u - cx) / fx, (v - cy) / fy, 1.0f};
        const Vec3 rp = rotate(dq_cam, ray);
        if (rp.z < 1e-3f) { obs.keypoints[i].depth_hint = lens_prior_m; continue; }
        const float u_rot = fx * rp.x / rp.z + cx;
        const float v_rot = fy * rp.y / rp.z + cy;

        // Subtract the rotational flow; only the translational residual carries
        // depth (camera-translation parallax).
        const float tdx = flow_[2 * i]     - (u_rot - u);
        const float tdy = flow_[2 * i + 1] - (v_rot - v);
        const float disparity = std::sqrt(tdx * tdx + tdy * tdy);
        if (disparity < cfg_.min_disparity_px) {
            obs.keypoints[i].depth_hint = lens_prior_m; // pure rotation / too far
            continue;
        }
        const float imu_depth = focal * b / disparity;
        obs.keypoints[i].depth_hint =
            kalman_fuse(lens_prior_m, imu_depth, cfg_.sigma_lens, cfg_.sigma_imu);
    }
}

} // namespace mec
