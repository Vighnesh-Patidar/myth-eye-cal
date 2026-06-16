#pragma once

// Core data types for Myth-Eye-Cal, per ARCHITECTURE.md §3, §4, §5.
// Pure C++17, no Android / MithAtomas dependencies.

#include <array>
#include <cstdint>

namespace mec {

// Number of keypoints the fusion core operates on. The pose estimator may
// produce up to 33 (MediaPipe), but fusion / broadcast use the 17-keypoint
// skeleton (§4.5, §6.2). 33-keypoint poses are downsampled to these 17 via
// kMediapipe33To17 (see keypoint_projector.h).
inline constexpr int kNumKeypoints = 17;

// Wire schema id for the UserStateVector payload ("ME"), §4.5.
inline constexpr uint16_t kMecSchemaId = 0x4D45;

// --- LOS state machine (§3.1) ---------------------------------------------
enum class LOSState : uint8_t {
    NO_TARGET = 0, // camera sees nothing / confidence below threshold
    ACQUIRING = 1, // target detected, depth not yet stable
    TRACKING  = 2, // target tracked, observations being broadcast
    OCCLUDED  = 3, // target was tracked, temporarily lost
};

// --- Observer-side types (§4.2) -------------------------------------------
struct Keypoint {
    uint8_t id = 0;
    float   x = 0.0f, y = 0.0f; // normalised image coordinates [0,1]
    float   depth_hint = 0.0f;  // monocular / temporal-stereo depth, metres
    float   confidence = 0.0f;  // per-keypoint visibility confidence [0,1]
};

struct PoseObservation {
    std::array<Keypoint, 33> keypoints{};
    uint8_t  keypoint_count = 0;
    uint32_t frame_id = 0;
    // Synced-clock time. DOUBLE, not float: an absolute synced-clock value in
    // seconds (~1e9) is unrepresentable at float32 precision (§ Design Review).
    double   timestamp_s = 0.0;
};

// --- World-frame keypoint (§4.4) ------------------------------------------
struct WorldKeypoint {
    uint8_t id = 0;
    float   wx = 0.0f, wy = 0.0f, wz = 0.0f; // world-frame position, metres
    // 1-sigma lateral uncertainty radius, metres (perpendicular to the view
    // ray). With the anisotropic fusion path (§15.3), the error along the ray
    // is given separately by depth_uncertainty.
    float   uncertainty_r = 0.0f;
    // §15.3 anisotropic model: unit view ray (camera -> point, world frame) and
    // the 1-sigma error ALONG it. rx=ry=rz=0 means "no ray" -> fused
    // isotropically using uncertainty_r. depth_uncertainty is typically >>
    // uncertainty_r for a monocular / temporal-stereo observation.
    float   rx = 0.0f, ry = 0.0f, rz = 0.0f;
    float   depth_uncertainty = 0.0f;
    float   confidence = 0.0f;
    double  timestamp_s = 0.0;
};

// --- Wire payload broadcast via UserStateVector (§4.5) --------------------
// Fits in UserStateVector's 128-byte payload, verified by static_assert below.
//
//   header        : schema_id(2) + keypoint_count(1) + frame_id(2) +
//                   timestamp_ms(4)                              =  9 bytes
//   keypoints[17] : 17 x (int16 wx,wy,wz [6] + uint8 conf [1])   = 119 bytes
//                                                          total = 128 bytes
//
// Keypoint id is IMPLICIT (array index 0..16) to save 17 bytes vs an explicit
// id field. World coords are fixed-point 1cm resolution, range +-327m.
// frame_id is uint16 (wraps every 65536 frames, ~36 min at 30fps - treat as
// circular). timestamp_ms is uint32 ms since session epoch (49-day range).
#pragma pack(push, 1)
struct KeypointFramePayload {
    uint16_t schema_id = kMecSchemaId;
    uint8_t  keypoint_count = 0;
    uint16_t frame_id = 0;
    uint32_t timestamp_ms = 0;
    struct PackedKeypoint {
        int16_t wx = 0, wy = 0, wz = 0; // fixed point, 1cm resolution
        uint8_t confidence = 0;         // 0-255 mapped from [0,1]
    } keypoints[kNumKeypoints];
};
#pragma pack(pop)

static_assert(sizeof(KeypointFramePayload) == 128,
              "KeypointFramePayload must be exactly 128 bytes (UserStateVector payload)");

// Fixed-point pack/unpack helpers for the wire payload (1cm resolution).
inline int16_t pack_metres(float m) {
    const float cm = m * 100.0f;
    if (cm > 32767.0f) return 32767;
    if (cm < -32768.0f) return -32768;
    return static_cast<int16_t>(cm < 0 ? cm - 0.5f : cm + 0.5f);
}
inline float unpack_metres(int16_t cm) { return static_cast<float>(cm) * 0.01f; }

inline uint8_t pack_confidence(float c) {
    if (c <= 0.0f) return 0;
    if (c >= 1.0f) return 255;
    return static_cast<uint8_t>(c * 255.0f + 0.5f);
}
inline float unpack_confidence(uint8_t c) { return static_cast<float>(c) / 255.0f; }

} // namespace mec
