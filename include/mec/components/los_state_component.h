#pragma once

// LOSStateComponent — hot component, owned by ObserverActivationSystem (§8).

#include "mec/types.h"
#include "mec/ecs/world.h"

namespace mec {

struct LOSStateComponent : mec::HotComponent<LOSStateComponent> {
    LOSState state = LOSState::NO_TARGET;
    float    confidence = 0.0f;
    uint32_t frame_id = 0;
};

} // namespace mec
