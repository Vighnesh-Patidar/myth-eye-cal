#pragma once

// IMUIntegrator — strapdown dead-reckoning integrator (ARCHITECTURE.md §4.3).
// Gyro integrates to orientation; gravity-compensated accelerometer integrates
// to velocity and displacement. consume() returns the IMUFrame (baseline +
// orientation) for one camera-frame interval and resets the displacement so
// position drift does not accumulate across frames (§13).
//
// On Android this is fed by SensorManager at 200Hz; here samples are injected
// directly, so the dead-reckoning math is testable on Linux without hardware.

#include "mec/math.h"
#include "mec/observer/frame.h"

namespace mec {

struct IMUConfig {
    float gravity = 9.81f; // m/s^2, world -Z
};

class IMUIntegrator {
public:
    explicit IMUIntegrator(IMUConfig cfg = IMUConfig{});

    // One IMU sample (device/body frame). accel_body is specific force in
    // m/s^2 (includes the +g reaction when stationary); gyro_body is angular
    // velocity in rad/s; dt is the sample interval in seconds.
    void integrate(const Vec3& accel_body, const Vec3& gyro_body, float dt);

    // Summarise motion since the last consume() and reset the displacement.
    // Velocity and orientation are retained (continuous physical state).
    IMUFrame consume(float timestamp_s);

    void reset();                              // zero velocity + displacement
    Quat orientation() const { return q_; }
    void set_orientation(const Quat& q) { q_ = q.normalized(); }
    Vec3 velocity() const { return v_; }
    Vec3 displacement() const { return p_; }

private:
    IMUConfig cfg_;
    Quat q_{1.0f, 0.0f, 0.0f, 0.0f};
    Vec3 v_{};
    Vec3 p_{};
};

} // namespace mec
