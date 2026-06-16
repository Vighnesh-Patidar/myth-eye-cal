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

// Scalar IMU summary for one inter-frame interval (§4.3). Produced by
// IMUIntegrator (or injected via FrameMeta on non-Android targets) and consumed
// by TemporalStereoDepth.
struct IMUFrame {
    float baseline_m = 0.0f;                          // distance moved since last frame
    float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f; // orientation at capture
    // Inter-frame rotation since the previous frame, body frame: maps a
    // direction in the current frame to the previous frame (§15.7 de-rotation).
    // Identity by default (no rotation / not provided).
    float dqw = 1.0f, dqx = 0.0f, dqy = 0.0f, dqz = 0.0f;
    // Unit translation direction since the previous frame, body frame
    // (§15.7 subject-motion gating). Zero = unknown -> epipolar gating disabled.
    float tdx = 0.0f, tdy = 0.0f, tdz = 0.0f;
    float timestamp_s = 0.0f;
};

} // namespace mec
