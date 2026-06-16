#include "mec/observer/imu_integrator.h"

#include <cmath>

namespace mec {

IMUIntegrator::IMUIntegrator(IMUConfig cfg) : cfg_(cfg) {}

void IMUIntegrator::integrate(const Vec3& accel_body, const Vec3& gyro_body, float dt) {
    // Orientation: integrate body-frame angular velocity via the exponential
    // map (exact for a constant axis over dt). q_{k+1} = q_k ⊗ Δq.
    const float w = norm(gyro_body);
    if (w > 1e-9f) {
        const float half = 0.5f * w * dt;
        const float s = std::sin(half) / w; // sin(half)/|w| -> axis*sin(half)
        const Quat dq{std::cos(half), gyro_body.x * s, gyro_body.y * s, gyro_body.z * s};
        q_ = mul(q_, dq).normalized();
    }

    // Specific force -> world-frame linear acceleration: a = R·f + g_world,
    // with g_world = (0,0,-g). A stationary device reads f=(0,0,+g) -> a=0.
    Vec3 a_world = rotate(q_, accel_body);
    a_world.z -= cfg_.gravity;

    // Kinematic update (exact for constant acceleration over dt).
    p_ = p_ + v_ * dt + a_world * (0.5f * dt * dt);
    v_ = v_ + a_world * dt;
}

IMUFrame IMUIntegrator::consume(float timestamp_s) {
    IMUFrame f;
    f.baseline_m = norm(p_);
    const Quat q = q_.normalized();
    f.qw = q.w; f.qx = q.x; f.qy = q.y; f.qz = q.z;

    // Inter-frame rotation since the last consume(), mapping current-frame
    // directions to the previous frame: dq = q_prev^{-1} ⊗ q_curr (§15.7).
    const Quat dq = mul(conj(prev_consumed_q_), q).normalized();
    f.dqw = dq.w; f.dqx = dq.x; f.dqy = dq.y; f.dqz = dq.z;

    // Unit translation direction in the body frame (for epipolar gating, §15.7).
    // p_ is the world-frame displacement; rotate it into the body frame.
    const float pn = norm(p_);
    if (pn > 1e-6f) {
        const Vec3 dir_body = rotate(conj(q), p_ * (1.0f / pn));
        f.tdx = dir_body.x; f.tdy = dir_body.y; f.tdz = dir_body.z;
    }
    f.timestamp_s = timestamp_s;

    p_ = Vec3{}; // displacement resets each camera frame; velocity + orientation persist
    prev_consumed_q_ = q;
    return f;
}

void IMUIntegrator::reset() {
    v_ = Vec3{};
    p_ = Vec3{};
}

} // namespace mec
