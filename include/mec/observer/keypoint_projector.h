#pragma once

// KeypointProjector - projects 2D image keypoints + depth into the shared
// world frame. ARCHITECTURE.md §4.4.
//
//   image (u,v) + depth d
//        -> camera intrinsics  -> camera-frame ray
//        -> camera->body extrinsic
//        -> orientation quaternion -> body/world rotation
//        -> + position offset   -> absolute world position

#include "mec/math.h"
#include "mec/types.h"
#include <array>

namespace mec {

// Pinhole intrinsics, in pixels. Keypoints arrive as normalised [0,1] image
// coordinates and are scaled to pixels via image_w / image_h.
struct CameraIntrinsics {
    float fx = 0.0f, fy = 0.0f; // focal length, pixels
    float cx = 0.0f, cy = 0.0f; // principal point, pixels
    int   image_w = 0, image_h = 0;
};

// Phone pose in the world frame.
struct NodePose {
    Vec3 position;       // PositionComponent, metres (world frame)
    Quat orientation;    // OrientationComponent, body->world rotation
};

class KeypointProjector {
public:
    KeypointProjector() = default;

    // Fixed camera->body extrinsic (phone-specific). Defaults to identity
    // (camera optical axis aligned with body +Z, looking forward).
    void set_camera_to_body(const Quat& q, const Vec3& t) {
        cam_to_body_rot_ = q.normalized();
        cam_to_body_trans_ = t;
    }

    // Project one keypoint. Returns the world-frame point with a confidence-
    // and depth-scaled uncertainty radius.
    WorldKeypoint project(const Keypoint& kp,
                          const CameraIntrinsics& intr,
                          const NodePose& pose,
                          double timestamp_s) const;

private:
    Quat cam_to_body_rot_{};   // identity
    Vec3 cam_to_body_trans_{};
};

// MediaPipe 33-landmark -> 17-keypoint skeleton index map (§4.5, §6.2).
// Index i of the output 17-skeleton reads MediaPipe landmark
// kMediapipe33To17[i]. Order: nose, eyes, ears, shoulders, elbows, wrists,
// hips, knees, ankles.
inline constexpr std::array<uint8_t, kNumKeypoints> kMediapipe33To17 = {
    0,  // nose
    2, 5,   // left eye, right eye
    7, 8,   // left ear, right ear
    11, 12, // left shoulder, right shoulder
    13, 14, // left elbow, right elbow
    15, 16, // left wrist, right wrist
    23, 24, // left hip, right hip
    25, 26, // left knee, right knee
    27,     // left ankle  (slot 16; right ankle 28 dropped to fit 17)
};

} // namespace mec
