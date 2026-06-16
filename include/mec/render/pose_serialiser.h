#pragma once

// PoseSerialiser - serialises a fused pose into the WebSocket "pose_frame"
// JSON (ARCHITECTURE.md §6.3). Header-only; hand-rolled JSON so the fusion
// core has no nlohmann/json dependency (that is only needed by the WS server).

#include "mec/types.h"
#include <array>
#include <string>

namespace mec {

struct FusedPose {
    std::array<WorldKeypoint, kNumKeypoints> keypoints{};
    double   timestamp_s = 0.0;
    uint8_t  observer_count = 0;
    float    mean_confidence = 0.0f;
    bool     is_valid = false;
};

inline std::string serialise_pose(const FusedPose& pose) {
    auto num = [](double v) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.3f", v);
        return std::string(buf);
    };

    std::string out = "{\"type\":\"pose_frame\",\"timestamp_s\":";
    out += num(pose.timestamp_s);
    out += ",\"observer_count\":" + std::to_string(pose.observer_count);
    out += ",\"keypoints\":[";
    for (int i = 0; i < kNumKeypoints; ++i) {
        const WorldKeypoint& k = pose.keypoints[i];
        if (i) out += ',';
        out += "{\"id\":" + std::to_string(k.id);
        out += ",\"wx\":" + num(k.wx);
        out += ",\"wy\":" + num(k.wy);
        out += ",\"wz\":" + num(k.wz);
        out += ",\"conf\":" + num(k.confidence);
        out += '}';
    }
    out += "]}";
    return out;
}

} // namespace mec
