#pragma once

// CameraIntrinsicsComponent — cold component, set at init (§8). Carries the
// pinhole intrinsics plus the fixed camera->body extrinsic (§4.4).

#include "mec/math.h"
#include "mec/observer/keypoint_projector.h"
#include "mith/atomas.h"

namespace mec {

struct CameraIntrinsicsComponent : mith::ColdComponent<CameraIntrinsicsComponent> {
    CameraIntrinsics intrinsics{};
    Quat             cam_to_body_rot{};   // identity default
    Vec3             cam_to_body_trans{};
};

} // namespace mec
