#pragma once

// ObserverMetricsComponent — cold component, written by ObserverPipeline (§8).

#include "mec/ecs/world.h"

namespace mec {

struct ObserverMetricsComponent : mec::ColdComponent<ObserverMetricsComponent> {
    float fps = 0.0f;
    float mean_inference_latency_ms = 0.0f;
    float drop_rate = 0.0f;
};

} // namespace mec
