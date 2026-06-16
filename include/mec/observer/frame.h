#pragma once

// Frame / FrameMeta — camera capture types (ARCHITECTURE.md §4.1). For the
// temporal-stereo depth path only the Y (luma) plane is needed.

#include <cstdint>

namespace mec {

struct Frame {
    const uint8_t* data = nullptr; // Y plane (grayscale), row-major, width*height
    int     width = 0;
    int     height = 0;
    int64_t timestamp_ns = 0;
};

struct FrameMeta {
    float focal_length_px = 0.0f;  // focal_length_mm / pixel_size_mm
    float focus_distance_m = 0.0f; // 1 / LENS_FOCUS_DISTANCE (lens prior)
    float imu_baseline_m = 0.0f;   // distance moved since last frame
    float imu_qw = 1.0f, imu_qx = 0.0f, imu_qy = 0.0f, imu_qz = 0.0f;
};

} // namespace mec
