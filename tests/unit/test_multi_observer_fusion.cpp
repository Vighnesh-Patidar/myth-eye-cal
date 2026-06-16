#include "mec/fusion/multi_observer_fusion.h"
#include "test_util.h"

#include <vector>

using namespace mec;

static WorldKeypoint kp(float x, float y, float z, float sigma, float conf) {
    WorldKeypoint k;
    k.wx = x; k.wy = y; k.wz = z;
    k.uncertainty_r = sigma; k.confidence = conf;
    return k;
}

int main() {
    // Empty input -> no fusion.
    {
        WorldKeypoint out;
        CHECK(!MultiObserverFusion::fuse({}, out));
    }

    // Single observation -> passthrough position.
    {
        WorldKeypoint out;
        CHECK(MultiObserverFusion::fuse({kp(1, 2, 3, 0.05f, 0.9f)}, out));
        CHECK_NEAR(out.wx, 1.0, 1e-5);
        CHECK_NEAR(out.wy, 2.0, 1e-5);
        CHECK_NEAR(out.wz, 3.0, 1e-5);
    }

    // Two equal-weight observations -> midpoint.
    {
        WorldKeypoint out;
        std::vector<WorldKeypoint> v = {kp(0, 0, 0, 0.1f, 0.8f),
                                        kp(2, 0, 0, 0.1f, 0.8f)};
        CHECK(MultiObserverFusion::fuse(v, out));
        CHECK_NEAR(out.wx, 1.0, 1e-4);
    }

    // Lower-uncertainty observation dominates the fused position.
    {
        WorldKeypoint out;
        std::vector<WorldKeypoint> v = {kp(0, 0, 0, 0.01f, 0.9f),   // tight
                                        kp(10, 0, 0, 0.50f, 0.9f)}; // loose
        CHECK(MultiObserverFusion::fuse(v, out));
        CHECK(out.wx < 0.5); // pulled strongly toward the tight observation
    }

    // Fused uncertainty shrinks below the best single observer (1/sqrt(N)).
    {
        WorldKeypoint out1, out4;
        MultiObserverFusion::fuse({kp(0, 0, 0, 0.10f, 0.9f)}, out1);
        std::vector<WorldKeypoint> four(4, kp(0, 0, 0, 0.10f, 0.9f));
        MultiObserverFusion::fuse(four, out4);
        CHECK(out4.uncertainty_r < out1.uncertainty_r);
        // 4 equal observers -> sigma/2.
        CHECK_NEAR(out4.uncertainty_r, out1.uncertainty_r / 2.0, 1e-3);
    }

    RUN_TESTS_RETURN();
}
