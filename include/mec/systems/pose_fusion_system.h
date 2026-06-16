#pragma once

// PoseFusionSystem (§9, 20-30Hz). For each keypoint, fuses the aggregated
// observations (weighted least squares) and folds the result into that
// keypoint's Kalman filter. Writes PoseStateComponent metadata; the smoothed
// keypoint positions are produced by KalmanPredictSystem at render rate.

#include "mec/components/internal_components.h"
#include "mec/components/pose_state_component.h"
#include "mec/fusion/multi_observer_fusion.h"
#include "mith/atomas.h"

namespace mec {

class PoseFusionSystem : public mith::System {
public:
    const char* name() const override { return "PoseFusionSystem"; }
    double      rate_hz() const override { return 25.0; }

    void update(mith::World& w, double) override {
        const mith::EntityId e = w.local();
        auto* agg = w.get<AggregatedObservationsComponent>(e);
        if (!agg) return;

        auto& bank = w.get_or_add<KalmanBankComponent>(e);
        auto& pose = w.get_or_add<PoseStateComponent>(e);

        int updated = 0;
        for (int k = 0; k < kNumKeypoints; ++k) {
            WorldKeypoint fused;
            // Anisotropic fusion (§15.3); isotropic observations (no view ray)
            // are handled identically to the scalar path within the same call.
            if (MultiObserverFusion::fuse_anisotropic(agg->per_keypoint[k], fused)) {
                fused.id = static_cast<uint8_t>(k);
                bank.trackers[k].update(fused);
                ++updated;
            }
        }
        if (updated > 0) {
            pose.observer_count = agg->observer_count;
            pose.last_update_s = w.now_s();
            pose.is_valid = bank.trackers[0].is_initialised();
        }
    }
};

} // namespace mec
