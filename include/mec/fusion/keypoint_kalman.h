#pragma once

// KeypointKalmanTracker - constant-velocity Kalman filter for a single world-
// frame keypoint. ARCHITECTURE.md §5.3.
//
// State: [pos, vel] per axis, three independent 1D filters (x, y, z). This is
// equivalent to the 6-vector [wx,wy,wz,vx,vy,vz] in the doc but decoupled per
// axis, which is exact for a diagonal process/measurement model and cheaper.
//
// update(): fold in a fused observation (called at fusion rate, 20-30Hz).
// predict(): forward-project to an arbitrary time WITHOUT mutating state
//            (called at render rate, 60Hz, between fusion updates).

#include "mec/types.h"

namespace mec {

class KeypointKalmanTracker {
public:
    // Process noise as a max-acceleration std-dev (m/s^2), tuned for human
    // movement (~5 m/s^2 per §5.3). Higher -> filter trusts measurements more.
    // Not explicit: also serves as the default constructor (e.g. std::array
    // value-init in KalmanBankComponent).
    KeypointKalmanTracker(float process_accel_std = 5.0f);

    void update(const WorldKeypoint& fused_obs);

    // Forward prediction to timestamp_s. Const: does not advance the filter.
    // The returned keypoint's confidence decays with time since last update.
    WorldKeypoint predict(double timestamp_s) const;

    bool  is_initialised() const { return initialised_; }
    // Confidence at the last update, before any decay.
    float confidence() const { return last_confidence_; }

private:
    struct Axis1D {
        float pos = 0.0f, vel = 0.0f;     // state estimate
        float p00 = 1.0f, p01 = 0.0f;     // covariance (symmetric 2x2)
        float p10 = 0.0f, p11 = 1.0f;
        void predict_inplace(float dt, float q_accel_var);
        void update_inplace(float z, float r);
        float predict_pos(float dt) const { return pos + vel * dt; }
    };

    Axis1D ax_, ay_, az_;
    float  process_accel_var_;     // (process_accel_std)^2
    double last_update_s_ = 0.0;
    float  last_confidence_ = 0.0f;
    uint8_t id_ = 0;
    bool   initialised_ = false;

    // Confidence half-life (s) when no updates arrive. After this long with no
    // update, predicted confidence is halved (§5.3 "decays when updates stop").
    static constexpr double kConfidenceHalfLifeS = 0.5;
};

} // namespace mec
