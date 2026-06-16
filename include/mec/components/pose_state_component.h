#pragma once

// PoseStateComponent — cold component, output of the fusion pipeline (§5.4).
// Written by PoseFusionSystem (metadata) + KalmanPredictSystem (keypoints);
// read by RenderSerialiserSystem.

#include "mec/types.h"
#include "mith/atomas.h"
#include <array>

namespace mec {

struct PoseStateComponent : mith::ColdComponent<PoseStateComponent> {
    std::array<WorldKeypoint, kNumKeypoints> keypoints{};
    double  last_update_s = 0.0;
    uint8_t observer_count = 0;   // how many LOS nodes contributed
    float   mean_confidence = 0.0f;
    bool    is_valid = false;
};

} // namespace mec
