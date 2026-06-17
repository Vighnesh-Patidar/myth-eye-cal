#pragma once

// KalmanPredictSystem (§9, 60Hz). Runs between fusion updates: forward-projects
// every keypoint filter to the current time so the renderer stays smooth even
// though fusion arrives at 20-30Hz. Writes the predicted keypoints into
// PoseStateComponent.

#include "mec/components/internal_components.h"
#include "mec/components/pose_state_component.h"
#include "mec/ecs/world.h"

namespace mec {

class KalmanPredictSystem : public mec::System {
public:
    const char* name() const override { return "KalmanPredictSystem"; }
    double      rate_hz() const override { return 60.0; }

    void update(mec::World& w, double) override {
        const mec::EntityId e = w.local();
        auto* bank = w.get<KalmanBankComponent>(e);
        auto* pose = w.get<PoseStateComponent>(e);
        if (!bank || !pose || !pose->is_valid) return;

        const double now = w.now_s();
        float csum = 0.0f;
        for (int k = 0; k < kNumKeypoints; ++k) {
            pose->keypoints[k] = bank->trackers[k].predict(now);
            csum += pose->keypoints[k].confidence;
        }
        pose->mean_confidence = csum / kNumKeypoints;
    }
};

} // namespace mec
