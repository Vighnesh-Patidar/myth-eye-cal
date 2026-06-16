#pragma once

// ObserverActivationSystem (§9, 10Hz). Reads the local node's latest pose
// observation, runs the LOS hysteresis state machine, and writes
// LOSStateComponent. Mirrors the state into the core BehaviourStateComponent
// so neighbours can see this node's LOS (§3.3).

#include "mec/components/internal_components.h"
#include "mec/components/los_state_component.h"
#include "mec/observer/los_detector.h"
#include "mith/atomas.h"

namespace mec {

class ObserverActivationSystem : public mith::System {
public:
    const char* name() const override { return "ObserverActivationSystem"; }
    double      rate_hz() const override { return 10.0; }

    void update(mith::World& w, double) override {
        const mith::EntityId e = w.local();
        auto* obsc = w.get<LatestObservationComponent>(e);
        if (!obsc) return; // node has no observer pipeline feeding it

        auto& los = w.get_or_add<LOSStateComponent>(e);
        los.state = detector_.update(obsc->obs, obsc->depth_stable);
        los.frame_id = obsc->obs.frame_id;

        const int n = (obsc->obs.keypoint_count < 33) ? obsc->obs.keypoint_count : 33;
        float sum = 0.0f;
        for (int i = 0; i < n; ++i) sum += obsc->obs.keypoints[i].confidence;
        los.confidence = n ? sum / static_cast<float>(n) : 0.0f;

        if (auto* bs = w.get<mith::BehaviourStateComponent>(e))
            bs->los_state = static_cast<uint8_t>(los.state);
    }

private:
    LOSDetector detector_;
};

} // namespace mec
