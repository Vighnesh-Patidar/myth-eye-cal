#include "mec/observer/keypoint_projector.h"
#include "test_util.h"

#include <cmath>

using namespace mec;

static CameraIntrinsics intr() {
    CameraIntrinsics c;
    c.fx = 500.0f; c.fy = 500.0f; c.cx = 320.0f; c.cy = 240.0f;
    c.image_w = 640; c.image_h = 480;
    return c;
}

static Keypoint at(float nx, float ny, float depth) {
    Keypoint k;
    k.id = 5; k.x = nx; k.y = ny; k.depth_hint = depth; k.confidence = 0.9f;
    return k;
}

int main() {
    KeypointProjector proj;
    const CameraIntrinsics c = intr();

    // Centre pixel, identity pose -> straight out along +Z at the given depth.
    {
        NodePose pose; // identity orientation, zero position
        WorldKeypoint w = proj.project(at(0.5f, 0.5f, 3.0f), c, pose, 1.0);
        CHECK_NEAR(w.wx, 0.0, 1e-4);
        CHECK_NEAR(w.wy, 0.0, 1e-4);
        CHECK_NEAR(w.wz, 3.0, 1e-4);
        CHECK(w.id == 5);
    }

    // Pixel offset right by 0.5*fx at depth 2 -> camera-frame x = 1.0.
    {
        const float nx = (c.cx + 0.5f * c.fx) / c.image_w; // u = cx + 250
        NodePose pose;
        WorldKeypoint w = proj.project(at(nx, 0.5f, 2.0f), c, pose, 1.0);
        CHECK_NEAR(w.wx, 1.0, 1e-4);
        CHECK_NEAR(w.wz, 2.0, 1e-4);
    }

    // Node position offset translates the world point.
    {
        const float nx = (c.cx + 0.5f * c.fx) / c.image_w;
        NodePose pose;
        pose.position = Vec3{10.0f, 0.0f, 0.0f};
        WorldKeypoint w = proj.project(at(nx, 0.5f, 2.0f), c, pose, 1.0);
        CHECK_NEAR(w.wx, 11.0, 1e-4);
    }

    // 90-degree yaw about Z rotates camera-frame (1,0,2) -> world (0,1,2).
    {
        const float nx = (c.cx + 0.5f * c.fx) / c.image_w;
        NodePose pose;
        const float h = std::sqrt(0.5f); // cos45 = sin45
        pose.orientation = Quat{h, 0.0f, 0.0f, h}; // +90 deg about Z
        WorldKeypoint w = proj.project(at(nx, 0.5f, 2.0f), c, pose, 1.0);
        CHECK_NEAR(w.wx, 0.0, 1e-4);
        CHECK_NEAR(w.wy, 1.0, 1e-4);
        CHECK_NEAR(w.wz, 2.0, 1e-4);
    }

    // Higher confidence -> tighter uncertainty radius.
    {
        NodePose pose;
        Keypoint hi = at(0.5f, 0.5f, 3.0f); hi.confidence = 0.95f;
        Keypoint lo = at(0.5f, 0.5f, 3.0f); lo.confidence = 0.30f;
        WorldKeypoint wh = proj.project(hi, c, pose, 1.0);
        WorldKeypoint wl = proj.project(lo, c, pose, 1.0);
        CHECK(wh.uncertainty_r < wl.uncertainty_r);
    }

    RUN_TESTS_RETURN();
}
