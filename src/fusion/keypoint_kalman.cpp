#include "mec/fusion/keypoint_kalman.h"

#include <algorithm>
#include <cmath>

namespace mec {

// --- 1D constant-velocity axis filter -------------------------------------

// Predict: x' = F x, P' = F P F^T + Q, with F = [[1, dt],[0, 1]].
// Q is the continuous white-acceleration model integrated over dt:
//   Q = q * [[dt^4/4, dt^3/2],[dt^3/2, dt^2]]
void KeypointKalmanTracker::Axis1D::predict_inplace(float dt, float q_accel_var) {
    // State.
    pos = pos + vel * dt;
    // vel unchanged.

    // Covariance: P' = F P F^T.
    const float p00n = p00 + dt * (p10 + p01) + dt * dt * p11;
    const float p01n = p01 + dt * p11;
    const float p10n = p10 + dt * p11;
    const float p11n = p11;

    // + Q.
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    const float dt4 = dt2 * dt2;
    p00 = p00n + q_accel_var * (dt4 * 0.25f);
    p01 = p01n + q_accel_var * (dt3 * 0.5f);
    p10 = p10n + q_accel_var * (dt3 * 0.5f);
    p11 = p11n + q_accel_var * dt2;
}

// Update with scalar position measurement z, variance r. H = [1, 0].
void KeypointKalmanTracker::Axis1D::update_inplace(float z, float r) {
    const float s = p00 + r;              // innovation variance
    if (s <= 0.0f) return;
    const float k0 = p00 / s;             // Kalman gain
    const float k1 = p10 / s;
    const float y = z - pos;              // innovation

    pos += k0 * y;
    vel += k1 * y;

    // P = (I - K H) P.
    const float p00n = (1.0f - k0) * p00;
    const float p01n = (1.0f - k0) * p01;
    const float p10n = p10 - k1 * p00;
    const float p11n = p11 - k1 * p01;
    p00 = p00n; p01 = p01n; p10 = p10n; p11 = p11n;
}

// --- Tracker ---------------------------------------------------------------

KeypointKalmanTracker::KeypointKalmanTracker(float process_accel_std)
    : process_accel_var_(process_accel_std * process_accel_std) {}

void KeypointKalmanTracker::update(const WorldKeypoint& obs) {
    const float r = std::max(obs.uncertainty_r * obs.uncertainty_r, 1e-6f);
    id_ = obs.id;
    last_confidence_ = obs.confidence;

    if (!initialised_) {
        ax_.pos = obs.wx; ay_.pos = obs.wy; az_.pos = obs.wz;
        ax_.vel = ay_.vel = az_.vel = 0.0f;
        // Seed covariance with measurement variance on position, large on vel.
        for (Axis1D* a : {&ax_, &ay_, &az_}) {
            a->p00 = r; a->p01 = 0.0f; a->p10 = 0.0f; a->p11 = 100.0f;
        }
        last_update_s_ = obs.timestamp_s;
        initialised_ = true;
        return;
    }

    float dt = static_cast<float>(obs.timestamp_s - last_update_s_);
    if (dt < 0.0f) dt = 0.0f; // out-of-order frame: predict no further

    ax_.predict_inplace(dt, process_accel_var_);
    ay_.predict_inplace(dt, process_accel_var_);
    az_.predict_inplace(dt, process_accel_var_);

    ax_.update_inplace(obs.wx, r);
    ay_.update_inplace(obs.wy, r);
    az_.update_inplace(obs.wz, r);

    last_update_s_ = obs.timestamp_s;
}

WorldKeypoint KeypointKalmanTracker::predict(double timestamp_s) const {
    WorldKeypoint out;
    out.id = id_;
    out.timestamp_s = timestamp_s;

    float dt = static_cast<float>(timestamp_s - last_update_s_);
    if (dt < 0.0f) dt = 0.0f;

    out.wx = ax_.predict_pos(dt);
    out.wy = ay_.predict_pos(dt);
    out.wz = az_.predict_pos(dt);

    // Predicted position uncertainty grows with dt (use x-axis as representative).
    Axis1D tmp = ax_;
    tmp.predict_inplace(dt, process_accel_var_);
    out.uncertainty_r = std::sqrt(std::max(tmp.p00, 0.0f));

    // Confidence decays exponentially with time since the last real update.
    const double decay = std::pow(0.5, static_cast<double>(dt) / kConfidenceHalfLifeS);
    out.confidence = static_cast<float>(last_confidence_ * decay);
    return out;
}

} // namespace mec
