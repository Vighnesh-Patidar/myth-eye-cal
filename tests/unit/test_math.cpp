// Tests for the header-only vector / quaternion math (math.h) used by the
// keypoint projector and the synthetic observer geometry.

#include "mec/math.h"
#include "test_util.h"

#include <cmath>

using namespace mec;

int main() {
    // --- Vec3 algebra ---
    const Vec3 a{1, 2, 3}, b{4, 5, 6};
    const Vec3 s = a + b;
    CHECK_NEAR(s.x, 5, 1e-6); CHECK_NEAR(s.y, 7, 1e-6); CHECK_NEAR(s.z, 9, 1e-6);
    const Vec3 d = b - a;
    CHECK_NEAR(d.x, 3, 1e-6); CHECK_NEAR(d.y, 3, 1e-6); CHECK_NEAR(d.z, 3, 1e-6);
    const Vec3 sc = a * 2.0f;
    CHECK_NEAR(sc.z, 6, 1e-6);
    CHECK_NEAR(dot(a, b), 32, 1e-5); // 4+10+18

    // Cross product is orthogonal to both inputs and right-handed.
    const Vec3 c = cross(Vec3{1, 0, 0}, Vec3{0, 1, 0});
    CHECK_NEAR(c.x, 0, 1e-6); CHECK_NEAR(c.y, 0, 1e-6); CHECK_NEAR(c.z, 1, 1e-6);
    CHECK_NEAR(dot(cross(a, b), a), 0, 1e-4);
    CHECK_NEAR(dot(cross(a, b), b), 0, 1e-4);

    CHECK_NEAR(norm(Vec3{3, 4, 0}), 5, 1e-6);

    // --- Quaternion ---
    // Degenerate normalize falls back to identity, never NaN.
    const Quat z = Quat{0, 0, 0, 0}.normalized();
    CHECK_NEAR(z.w, 1, 1e-6);
    const Quat n = Quat{0, 0, 3, 0}.normalized();
    CHECK_NEAR(n.x * n.x + n.y * n.y + n.z * n.z + n.w * n.w, 1.0, 1e-6);

    // Identity rotation leaves a vector unchanged.
    const Vec3 v0 = rotate(Quat{}, Vec3{1, 2, 3});
    CHECK_NEAR(v0.x, 1, 1e-6); CHECK_NEAR(v0.y, 2, 1e-6); CHECK_NEAR(v0.z, 3, 1e-6);

    // 90 deg about +Z maps +X -> +Y.
    const float h = std::sqrt(2.0f) / 2.0f;
    const Quat rz{h, 0, 0, h};
    const Vec3 rv = rotate(rz, Vec3{1, 0, 0});
    CHECK_NEAR(rv.x, 0, 1e-5); CHECK_NEAR(rv.y, 1, 1e-5); CHECK_NEAR(rv.z, 0, 1e-5);

    // q ⊗ conj(q) == identity for a unit quaternion.
    const Quat q = Quat{0.3f, 0.4f, 0.5f, 0.7f}.normalized();
    const Quat id = mul(q, conj(q));
    CHECK_NEAR(id.w, 1, 1e-5);
    CHECK_NEAR(id.x, 0, 1e-5); CHECK_NEAR(id.y, 0, 1e-5); CHECK_NEAR(id.z, 0, 1e-5);

    // Rotating then un-rotating returns the original vector.
    const Vec3 v{0.2f, -1.3f, 2.0f};
    const Vec3 back = rotate(conj(q), rotate(q, v));
    CHECK_NEAR(back.x, v.x, 1e-4);
    CHECK_NEAR(back.y, v.y, 1e-4);
    CHECK_NEAR(back.z, v.z, 1e-4);

    RUN_TESTS_RETURN();
}
