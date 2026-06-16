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
                                  float focal_length_px, float lens_prior_m) {
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

    for (int i = 0; i < n; ++i) {
        if (!status_[i]) { obs.keypoints[i].depth_hint = lens_prior_m; continue; }
        const float dx = flow_[2 * i], dy = flow_[2 * i + 1];
        const float disparity = std::sqrt(dx * dx + dy * dy);
        if (disparity < cfg_.min_disparity_px) {
            obs.keypoints[i].depth_hint = lens_prior_m; // target too far
            continue;
        }
        const float imu_depth = focal_length_px * b / disparity;
        obs.keypoints[i].depth_hint =
            kalman_fuse(lens_prior_m, imu_depth, cfg_.sigma_lens, cfg_.sigma_imu);
    }
}

} // namespace mec
