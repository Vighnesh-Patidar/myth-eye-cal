#include "mec/observer/keypoint_projector.h"

#include <algorithm>

namespace mec {

WorldKeypoint KeypointProjector::project(const Keypoint& kp,
                                         const CameraIntrinsics& intr,
                                         const NodePose& pose,
                                         double timestamp_s) const {
    // Normalised [0,1] -> pixels.
    const float u = kp.x * static_cast<float>(intr.image_w);
    const float v = kp.y * static_cast<float>(intr.image_h);
    const float d = kp.depth_hint;

    // Camera-frame ray via pinhole back-projection. Camera looks down +Z;
    // x right, y down (image convention).
    const Vec3 cam_ray{
        (u - intr.cx) / intr.fx * d,
        (v - intr.cy) / intr.fy * d,
        d,
    };

    // Camera -> body.
    const Vec3 body = rotate(cam_to_body_rot_, cam_ray) + cam_to_body_trans_;

    // Body -> world (rotate then translate by node position).
    const Vec3 world = rotate(pose.orientation, body) + pose.position;

    WorldKeypoint out;
    out.id = kp.id;
    out.wx = world.x;
    out.wy = world.y;
    out.wz = world.z;
    out.confidence = kp.confidence;
    out.timestamp_s = timestamp_s;

    // Uncertainty model (v0.1, isotropic scalar - see Design Review note in
    // types.h). Depth-proportional base error, inflated as confidence drops.
    // ~5cm at 3m for a confident keypoint, growing for low-confidence ones.
    const float depth_frac = 0.017f;                 // ~5cm / 3m
    const float conf = std::clamp(kp.confidence, 0.05f, 1.0f);
    out.uncertainty_r = (depth_frac * std::max(d, 0.1f)) / conf;
    return out;
}

} // namespace mec
