#include "mec/fusion/keypoint_kalman.h"
#include "test_util.h"

using namespace mec;

static WorldKeypoint obs(float x, float y, float z, double t, float sigma = 0.05f) {
    WorldKeypoint k;
    k.wx = x; k.wy = y; k.wz = z;
    k.uncertainty_r = sigma; k.confidence = 0.9f; k.timestamp_s = t;
    return k;
}

// Observation carrying an anisotropic diagonal covariance (sharp laterally in
// x,y; uncertain along z), as fuse_anisotropic would produce.
static WorldKeypoint covObs(float x, float y, float z, double t,
                            float s_lat, float s_depth) {
    WorldKeypoint k;
    k.wx = x; k.wy = y; k.wz = z;
    k.confidence = 0.9f; k.timestamp_s = t;
    k.cov_xx = s_lat * s_lat; k.cov_yy = s_lat * s_lat; k.cov_zz = s_depth * s_depth;
    k.uncertainty_r = s_lat; // RMS-ish summary, unused when cov is present
    return k;
}

int main() {
    // Uninitialised tracker.
    {
        KeypointKalmanTracker t;
        CHECK(!t.is_initialised());
    }

    // First update initialises to the measurement.
    {
        KeypointKalmanTracker t;
        t.update(obs(1, 2, 3, 0.0));
        CHECK(t.is_initialised());
        WorldKeypoint p = t.predict(0.0);
        CHECK_NEAR(p.wx, 1.0, 1e-3);
        CHECK_NEAR(p.wy, 2.0, 1e-3);
        CHECK_NEAR(p.wz, 3.0, 1e-3);
    }

    // Constant-velocity motion: tracker estimates velocity and predicts ahead.
    {
        KeypointKalmanTracker t;
        // Target moving at 1 m/s along x, sampled at 25Hz for ~1s.
        for (int i = 0; i <= 25; ++i) {
            const double time = i * 0.04;
            t.update(obs(static_cast<float>(time), 0, 0, time));
        }
        // Predict 0.1s past the last update (t=1.0) -> expect ~1.1m.
        WorldKeypoint p = t.predict(1.1);
        CHECK_NEAR(p.wx, 1.1, 0.05);
    }

    // Confidence decays when updates stop arriving.
    {
        KeypointKalmanTracker t;
        t.update(obs(0, 0, 0, 0.0));
        const float c0 = t.predict(0.0).confidence;
        const float c1 = t.predict(0.5).confidence; // one half-life later
        CHECK(c1 < c0);
        CHECK_NEAR(c1, c0 * 0.5, 0.05);
    }

    // §15.3: anisotropic covariance survives temporal filtering. Feed static
    // origin observations that are sharp in x,y (sigma 0.02m) but ambiguous in
    // z (sigma 0.5m). The steady-state position covariance must stay anisotropic
    // (depth >> lateral) — the old scalar filter would report it isotropic.
    {
        KeypointKalmanTracker t;
        for (int i = 0; i <= 40; ++i) {
            const double time = i * 0.04;
            t.update(covObs(0, 0, 0, time, /*s_lat=*/0.02f, /*s_depth=*/0.5f));
        }
        const WorldKeypoint p = t.predict(40 * 0.04);
        CHECK_NEAR(p.wx, 0.0, 0.02);
        CHECK_NEAR(p.wz, 0.0, 0.05);
        // Depth covariance stays far larger than lateral covariance.
        CHECK(p.cov_zz > p.cov_xx * 5.0f);
    }

    // §15.3: the gain is direction-dependent. After converging at the origin,
    // a single observation displaced equally in x (sharp) and z (uncertain)
    // moves the estimate MORE along the trusted lateral axis than along depth.
    {
        KeypointKalmanTracker t;
        double time = 0.0;
        for (int i = 0; i <= 40; ++i, time += 0.04)
            t.update(covObs(0, 0, 0, time, 0.02f, 0.5f));
        t.update(covObs(0.5f, 0.0f, 0.5f, time, 0.02f, 0.5f));
        const WorldKeypoint p = t.predict(time);
        CHECK(p.wx > 0.0f);            // moved toward the new lateral reading
        CHECK(p.wz > 0.0f);            // and (less) toward the new depth reading
        CHECK(p.wx > p.wz + 0.05f);    // but clearly more along the sharp axis
    }

    // Isotropic (scalar) path still reduces to independent per-axis filters:
    // a diagonal-only setup keeps cross-axis covariance at zero.
    {
        KeypointKalmanTracker t;
        for (int i = 0; i <= 10; ++i) t.update(obs(1, 2, 3, i * 0.04));
        const WorldKeypoint p = t.predict(10 * 0.04);
        CHECK_NEAR(p.cov_xy, 0.0, 1e-6);
        CHECK_NEAR(p.cov_xz, 0.0, 1e-6);
        CHECK_NEAR(p.cov_yz, 0.0, 1e-6);
    }

    RUN_TESTS_RETURN();
}
