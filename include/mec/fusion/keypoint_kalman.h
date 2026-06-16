#pragma once

// KeypointKalmanTracker - coupled 6-state (position + velocity) constant-
// velocity Kalman filter for a single world-frame keypoint. ARCHITECTURE.md
// §5.3 / §15.3.
//
// State: x = [px,py,pz, vx,vy,vz]. The measurement is a 3D position with a FULL
// 3x3 covariance R, so the anisotropic fused covariance from fuse_anisotropic()
// (§15.3 — sharp perpendicular to the view ray, uncertain along it) is carried
// THROUGH the temporal filter instead of being collapsed to one scalar. The
// gain is therefore direction-dependent: sharp axes are trusted, the depth
// axis is smoothed harder, and cross-axis correlations are preserved.
//
// When the fused observation carries no covariance (cov_xx <= 0) the filter
// falls back to R = uncertainty_r^2 * I (isotropic), which reduces EXACTLY to
// the previous three decoupled per-axis filters.
//
// update():  fold in a fused observation (called at fusion rate, 20-30Hz).
// predict(): forward-project to an arbitrary time WITHOUT mutating state
//            (render rate, 60Hz, between fusion updates). The returned
//            keypoint's confidence decays with time since the last update and
//            its cov_* carries the predicted position covariance.

#include "mec/types.h"

namespace mec {

class KeypointKalmanTracker {
public:
    // Process noise as a max-acceleration std-dev (m/s^2), tuned for human
    // movement (~5 m/s^2 per §5.3). Higher -> filter trusts measurements more.
    // Doubles as the default constructor (e.g. std::array value-init in
    // KalmanBankComponent).
    KeypointKalmanTracker(float process_accel_std = 5.0f);

    void update(const WorldKeypoint& fused_obs);

    // Forward prediction to timestamp_s. Const: does not advance the filter.
    WorldKeypoint predict(double timestamp_s) const;

    bool  is_initialised() const { return initialised_; }
    // Confidence at the last update, before any decay.
    float confidence() const { return last_confidence_; }

private:
    // 6x6 covariance held as symmetric 3x3 blocks: P = [[A, B],[Bᵀ, D]] with
    // A = position cov (sym), D = velocity cov (sym), B = pos/vel cross cov.
    // (Symmetry of P guarantees the lower block is Bᵀ, so it isn't stored.)
    // All 3x3 matrices are row-major.
    float p_[3] = {0.0f, 0.0f, 0.0f};   // position estimate
    float v_[3] = {0.0f, 0.0f, 0.0f};   // velocity estimate
    float A_[9] = {0.0f};               // position covariance
    float B_[9] = {0.0f};               // position-velocity cross covariance
    float D_[9] = {0.0f};               // velocity covariance

    float   process_accel_var_;         // (process_accel_std)^2
    double  last_update_s_ = 0.0;
    float   last_confidence_ = 0.0f;
    uint8_t id_ = 0;
    bool    initialised_ = false;

    // Confidence half-life (s) when no updates arrive (§5.3).
    static constexpr double kConfidenceHalfLifeS = 0.5;
};

} // namespace mec
