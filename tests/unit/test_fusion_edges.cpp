// Edge cases for MultiObserverFusion not covered by the happy-path tests:
//   - zero-confidence (zero-weight) inputs are skipped / reject the fusion
//   - fused confidence is the weight-weighted mean of inputs
//   - fuse_anisotropic emits a symmetric covariance the Kalman tracker consumes

#include "mec/fusion/multi_observer_fusion.h"
#include "test_util.h"

#include <vector>

using namespace mec;

static WorldKeypoint iso(float x, float y, float z, float sigma, float conf) {
    WorldKeypoint k;
    k.wx = x; k.wy = y; k.wz = z;
    k.uncertainty_r = sigma; k.confidence = conf;
    return k;
}
static WorldKeypoint aniso(float x, float y, float z, float rx, float ry, float rz,
                           float s_lat, float s_depth, float conf) {
    WorldKeypoint k;
    k.wx = x; k.wy = y; k.wz = z;
    k.rx = rx; k.ry = ry; k.rz = rz;
    k.uncertainty_r = s_lat; k.depth_uncertainty = s_depth; k.confidence = conf;
    return k;
}

int main() {
    // All-zero confidence -> all weights zero -> no usable fusion.
    {
        WorldKeypoint o;
        std::vector<WorldKeypoint> v = {iso(0, 0, 0, 0.1f, 0.0f),
                                        iso(1, 1, 1, 0.1f, 0.0f)};
        CHECK(!MultiObserverFusion::fuse(v, o));
        CHECK(!MultiObserverFusion::fuse_anisotropic(v, o));
    }

    // A zero-confidence observation is ignored; the good one passes through.
    {
        WorldKeypoint o;
        std::vector<WorldKeypoint> v = {iso(5, 5, 5, 0.1f, 0.0f),   // ignored
                                        iso(1, 2, 3, 0.1f, 0.9f)};  // used
        CHECK(MultiObserverFusion::fuse(v, o));
        CHECK_NEAR(o.wx, 1.0, 1e-4);
        CHECK_NEAR(o.wy, 2.0, 1e-4);
        CHECK_NEAR(o.wz, 3.0, 1e-4);
    }

    // Fused confidence is the weight-weighted mean (equal weights -> arithmetic
    // mean of the two confidences).
    {
        WorldKeypoint o;
        std::vector<WorldKeypoint> v = {iso(0, 0, 0, 0.1f, 0.6f),
                                        iso(0, 0, 0, 0.1f, 0.9f)};
        CHECK(MultiObserverFusion::fuse(v, o));
        CHECK_NEAR(o.confidence, 0.75, 0.05); // 0.6 & 0.9 weighted toward 0.9
    }

    // Anisotropic fusion emits a symmetric, positive covariance.
    {
        WorldKeypoint o;
        std::vector<WorldKeypoint> v = {
            aniso(0, 0, 0, 1, 0, 0, 0.02f, 0.5f, 0.9f),
            aniso(0, 0, 0, 0, 1, 0, 0.02f, 0.5f, 0.9f),
            aniso(0, 0, 0, 0, 0, 1, 0.02f, 0.5f, 0.9f),
        };
        CHECK(MultiObserverFusion::fuse_anisotropic(v, o));
        CHECK(o.cov_xx > 0.0f && o.cov_yy > 0.0f && o.cov_zz > 0.0f);
        // Symmetric storage: off-diagonals are tiny for orthogonal rays.
        CHECK_NEAR(o.cov_xy, 0.0, 1e-3);
        CHECK_NEAR(o.cov_xz, 0.0, 1e-3);
        CHECK_NEAR(o.cov_yz, 0.0, 1e-3);
        CHECK(o.uncertainty_r > 0.0f); // isotropic summary set
        CHECK(o.rx == 0.0f && o.ry == 0.0f && o.rz == 0.0f); // fused ray cleared
    }

    RUN_TESTS_RETURN();
}
