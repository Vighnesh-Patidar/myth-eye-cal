#pragma once

// KeypointAggregatorSystem (§9, 20-30Hz). Pulls UserStateVector entries from
// the UserNeighbourTable, filters them (§5.1), unpacks the world keypoints,
// and groups them per keypoint into the AggregatedObservationsComponent buffer.

#include "mec/components/internal_components.h"
#include "mec/types.h"
#include "mith/atomas.h"

#include <algorithm>
#include <cstring>

namespace mec {

class KeypointAggregatorSystem : public mith::System {
public:
    explicit KeypointAggregatorSystem(double fusion_window_s = 0.150)
        : window_s_(fusion_window_s) {}

    const char* name() const override { return "KeypointAggregatorSystem"; }
    double      rate_hz() const override { return 25.0; }

    void update(mith::World& w, double) override {
        const mith::EntityId e = w.local();
        auto& agg = w.get_or_add<AggregatedObservationsComponent>(e);
        agg.clear();
        const double now = w.now_s();
        int observers = 0;

        for (const auto& usv : w.user_neighbours.entries) {
            KeypointFramePayload pl;
            std::memcpy(&pl, usv.payload.data(), sizeof(pl));

            if (pl.schema_id != kMecSchemaId) continue;                       // §5.1
            if (usv.los_state != static_cast<uint8_t>(LOSState::TRACKING)) continue; // §5.1
            if (now - usv.recv_time_s > window_s_) continue;                  // §5.1
            ++observers;

            const int n = std::min<int>(pl.keypoint_count, kNumKeypoints);
            for (int i = 0; i < n; ++i) {
                WorldKeypoint wk;
                wk.id = static_cast<uint8_t>(i);
                wk.wx = unpack_metres(pl.keypoints[i].wx);
                wk.wy = unpack_metres(pl.keypoints[i].wy);
                wk.wz = unpack_metres(pl.keypoints[i].wz);
                wk.confidence = unpack_confidence(pl.keypoints[i].confidence);
                // The 128-byte wire payload carries confidence but NOT the
                // per-observation uncertainty (no room — §4.5). Reconstruct a
                // nominal sigma from confidence so the fuser can weight it.
                wk.uncertainty_r = 0.03f / std::max(wk.confidence, 0.05f);
                wk.timestamp_s = usv.recv_time_s;
                agg.per_keypoint[i].push_back(wk);
            }
        }
        agg.observer_count = static_cast<uint8_t>(observers);
        agg.window_end_s = now;
    }

private:
    double window_s_;
};

} // namespace mec
