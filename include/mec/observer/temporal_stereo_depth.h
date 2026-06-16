#pragma once

// TemporalStereoDepth — metric depth from a single moving camera (no depth ML
// model), ARCHITECTURE.md §4.3. The phone is a moving eye: IMU dead reckoning
// gives the baseline, Lucas-Kanade optical flow gives per-keypoint disparity,
// and depth = focal_length_px * baseline / disparity. That measurement is
// Kalman-fused with the lens-focus-distance prior.
//
//   imu_depth   = focal_length_px * baseline / disparity
//   depth_fused = kalman_fuse(lens_prior, imu_depth, sigma_lens, sigma_imu)
//
// Fallbacks (§4.3):
//   baseline  < 5mm   -> lens prior only (phone stationary)
//   baseline  > 500mm -> skip frame      (motion too fast, flow unreliable)
//   disparity < 0.5px -> lens prior only (target too far for reliable disparity)
//
// CAVEAT (§15.7): temporal stereo recovers depth from camera-translation
// parallax of STATIC points. A moving subject and inter-frame camera rotation
// both contaminate the disparity; see §15.7 for the planned correction.

#include "mec/observer/frame.h"
#include "mec/observer/lucas_kanade.h"
#include "mec/types.h"

#include <cstdint>

namespace mec {

// IMUFrame (the scalar IMU summary, §4.3) is defined in frame.h.

class TemporalStereoDepth {
public:
    struct Config {
        float sigma_lens = 0.20f;     // lens-prior 1-sigma, metres (§4.3)
        float sigma_imu = 0.05f;      // imu-depth 1-sigma, metres (§4.3)
        float min_baseline_m = 0.005f;
        float max_baseline_m = 0.500f;
        float min_disparity_px = 0.5f;
    };

    TemporalStereoDepth(int max_w, int max_h);
    TemporalStereoDepth(int max_w, int max_h, Config cfg);

    // Updates obs.keypoints[i].depth_hint in place from the prev->curr flow.
    void resolve(PoseObservation& obs, const Frame& prev, const Frame& curr,
                 const IMUFrame& imu, float focal_length_px, float lens_prior_m);

    // Single-measurement inverse-variance fusion (a static Kalman update).
    static float kalman_fuse(float prior, float meas,
                             float sigma_prior, float sigma_meas);

private:
    LucasKanade lk_;
    Config      cfg_;
    // Pre-allocated scratch (no heap in the hot path).
    float   pts_[33 * 2];
    float   flow_[33 * 2];
    uint8_t status_[33];
};

} // namespace mec
