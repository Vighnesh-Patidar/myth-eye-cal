#include "mec/fusion/keypoint_kalman.h"
#include "test_util.h"

using namespace mec;

static WorldKeypoint obs(float x, float y, float z, double t, float sigma = 0.05f) {
    WorldKeypoint k;
    k.wx = x; k.wy = y; k.wz = z;
    k.uncertainty_r = sigma; k.confidence = 0.9f; k.timestamp_s = t;
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

    RUN_TESTS_RETURN();
}
