#include "mec/fusion/multi_observer_fusion.h"
#include "test_util.h"

#include <cmath>
#include <vector>

using namespace mec;

// World keypoint with an anisotropic (ray + depth/lateral sigma) error model.
static WorldKeypoint aniso(float x, float y, float z,
                           float rx, float ry, float rz,
                           float sigma_lat, float sigma_depth, float conf = 1.0f) {
    WorldKeypoint k;
    k.wx = x; k.wy = y; k.wz = z;
    k.rx = rx; k.ry = ry; k.rz = rz;
    k.uncertainty_r = sigma_lat; k.depth_uncertainty = sigma_depth;
    k.confidence = conf;
    return k;
}

static WorldKeypoint iso(float x, float y, float z, float sigma, float conf = 1.0f) {
    WorldKeypoint k;
    k.wx = x; k.wy = y; k.wz = z;
    k.uncertainty_r = sigma; k.confidence = conf;
    return k;
}

int main() {
    // Empty -> no fusion.
    {
        WorldKeypoint o;
        CHECK(!MultiObserverFusion::fuse_anisotropic({}, o));
    }

    // Single observation -> recovers its position exactly.
    {
        WorldKeypoint o;
        CHECK(MultiObserverFusion::fuse_anisotropic(
            {aniso(1, 2, 3, 1, 0, 0, 0.02f, 0.5f)}, o));
        CHECK_NEAR(o.wx, 1.0, 1e-4);
        CHECK_NEAR(o.wy, 2.0, 1e-4);
        CHECK_NEAR(o.wz, 3.0, 1e-4);
    }

    // Isotropic observations (no ray) -> same weighted mean as scalar fuse.
    {
        WorldKeypoint o;
        std::vector<WorldKeypoint> v = {iso(0, 0, 0, 0.1f), iso(2, 0, 0, 0.1f)};
        CHECK(MultiObserverFusion::fuse_anisotropic(v, o));
        CHECK_NEAR(o.wx, 1.0, 1e-4);
    }

    // THE geometry bonus (§5.2 / §15.3): two observers at orthogonal angles.
    // True point at the origin.
    //  - Observer A views along +X: depth-ambiguous in X, sharp in Y,Z. It
    //    reports a position wrong by 0.3m ALONG its ray (X), correct in Y,Z.
    //  - Observer B views along +Y: depth-ambiguous in Y, sharp in X,Z. It
    //    reports a position wrong by 0.3m along its ray (Y), correct in X,Z.
    // Anisotropic fusion should take X from B and Y from A -> ~origin.
    {
        std::vector<WorldKeypoint> v = {
            aniso(0.3f, 0.0f, 0.0f, 1, 0, 0, 0.02f, 0.5f), // A: wrong in X (its depth)
            aniso(0.0f, 0.3f, 0.0f, 0, 1, 0, 0.02f, 0.5f), // B: wrong in Y (its depth)
        };
        WorldKeypoint o;
        CHECK(MultiObserverFusion::fuse_anisotropic(v, o));
        CHECK_NEAR(o.wx, 0.0, 0.01);
        CHECK_NEAR(o.wy, 0.0, 0.01);
        CHECK_NEAR(o.wz, 0.0, 0.01);

        // The naive isotropic mean would sit near (0.15, 0.15, 0) - far off.
        WorldKeypoint naive;
        MultiObserverFusion::fuse(
            {iso(0.3f, 0, 0, 0.1f), iso(0, 0.3f, 0, 0.1f)}, naive);
        const double aniso_err = std::sqrt(o.wx * o.wx + o.wy * o.wy + o.wz * o.wz);
        const double naive_err =
            std::sqrt(naive.wx * naive.wx + naive.wy * naive.wy + naive.wz * naive.wz);
        CHECK(aniso_err < 0.02);
        CHECK(naive_err > 0.15);
        CHECK(aniso_err < naive_err * 0.2); // anisotropic >5x better here
    }

    // Fused uncertainty shrinks below a single observer's lateral sigma.
    {
        WorldKeypoint one, four;
        MultiObserverFusion::fuse_anisotropic({aniso(0, 0, 0, 1, 0, 0, 0.1f, 0.1f)}, one);
        std::vector<WorldKeypoint> v(4, aniso(0, 0, 0, 1, 0, 0, 0.1f, 0.1f));
        MultiObserverFusion::fuse_anisotropic(v, four);
        CHECK(four.uncertainty_r < one.uncertainty_r);
    }

    RUN_TESTS_RETURN();
}
