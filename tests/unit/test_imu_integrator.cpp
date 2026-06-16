#include "mec/observer/imu_integrator.h"
#include "test_util.h"

#include <cmath>

using namespace mec;

static constexpr double kPi = 3.14159265358979323846;

int main() {
    const float dt = 0.005f; // 200Hz
    const Vec3 g_only{0.0f, 0.0f, 9.81f}; // stationary specific force

    // 1) Gyro pi/2 rad/s about +Z for 1s -> 90deg yaw; rotates +X to +Y.
    {
        IMUIntegrator imu;
        const float wz = static_cast<float>(kPi / 2.0);
        for (int i = 0; i < 200; ++i) imu.integrate(g_only, Vec3{0, 0, wz}, dt);
        const Quat q = imu.orientation();
        CHECK_NEAR(q.w, std::cos(kPi / 4.0), 1e-2);
        CHECK_NEAR(q.z, std::sin(kPi / 4.0), 1e-2);
        const Vec3 r = rotate(q, Vec3{1, 0, 0});
        CHECK_NEAR(r.x, 0.0, 1e-2);
        CHECK_NEAR(r.y, 1.0, 1e-2);

        // consume() reports the inter-frame rotation (§15.7): from identity to
        // the current 90deg yaw -> dq ~ 90deg about Z.
        const IMUFrame f = imu.consume(1.0f);
        CHECK_NEAR(f.dqw, std::cos(kPi / 4.0), 1e-2);
        CHECK_NEAR(f.dqz, std::sin(kPi / 4.0), 1e-2);
    }

    // 2) Stationary: gravity fully rejected -> ~zero baseline.
    {
        IMUIntegrator imu;
        for (int i = 0; i < 200; ++i) imu.integrate(g_only, Vec3{0, 0, 0}, dt);
        const IMUFrame f = imu.consume(1.0f);
        CHECK(f.baseline_m < 1e-3f);
    }

    // 3) Accelerate then coast: displacement resets each frame, velocity carries.
    {
        IMUIntegrator imu;
        // 2 m/s^2 along +X for 0.5s -> v=1 m/s, displacement 0.25m.
        for (int i = 0; i < 100; ++i) imu.integrate(Vec3{2.0f, 0, 9.81f}, Vec3{0, 0, 0}, dt);
        const IMUFrame fa = imu.consume(0.5f);
        CHECK_NEAR(fa.baseline_m, 0.25, 2e-3);
        CHECK_NEAR(imu.velocity().x, 1.0, 2e-3);

        // Coast (zero accel) at v=1 m/s for 0.1s -> baseline 0.1m.
        for (int i = 0; i < 20; ++i) imu.integrate(g_only, Vec3{0, 0, 0}, dt);
        const IMUFrame fb = imu.consume(0.6f);
        CHECK_NEAR(fb.baseline_m, 0.1, 2e-3);
        // Motion was purely +X -> reported translation direction ~ (1,0,0) (§15.7a).
        CHECK_NEAR(fb.tdx, 1.0, 1e-2);
        CHECK_NEAR(fb.tdy, 0.0, 1e-2);
    }

    // 4) reset() zeroes velocity + displacement (orientation kept).
    {
        IMUIntegrator imu;
        for (int i = 0; i < 50; ++i) imu.integrate(Vec3{3.0f, 0, 9.81f}, Vec3{0, 0, 0}, dt);
        CHECK(imu.velocity().x > 0.0f);
        imu.reset();
        CHECK_NEAR(imu.velocity().x, 0.0, 1e-6);
        CHECK_NEAR(imu.displacement().x, 0.0, 1e-6);
    }

    RUN_TESTS_RETURN();
}
