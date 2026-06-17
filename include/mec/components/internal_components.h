#pragma once

// Node-local scratch components — implementation-internal, NOT part of the
// swarm-replicated StateVector (so not in §8's table). They form the seams
// between systems and between the (deferred) observer pipeline and the ECS.

#include "mec/fusion/keypoint_kalman.h"
#include "mec/types.h"
#include "mec/ecs/world.h"

#include <array>
#include <vector>

namespace mec {

// Written by the observer pipeline (camera -> pose -> depth, v0.2); read by
// ObserverActivationSystem and KeypointBroadcastSystem. This is the seam where
// MediaPipe + TemporalStereoDepth will plug in.
struct LatestObservationComponent : mec::ColdComponent<LatestObservationComponent> {
    PoseObservation obs{};
    bool depth_stable = false; // set by the depth pipeline once depth settles
};

// Aggregator -> fusion buffer (§9 "internal buffer"). One observation list per
// keypoint, gathered from all current LOS neighbours within the fusion window.
struct AggregatedObservationsComponent : mec::ColdComponent<AggregatedObservationsComponent> {
    std::array<std::vector<WorldKeypoint>, kNumKeypoints> per_keypoint{};
    uint8_t observer_count = 0;
    double  window_end_s = 0.0;
    void clear() {
        for (auto& v : per_keypoint) v.clear();
        observer_count = 0;
    }
};

// Persistent per-keypoint Kalman filters, shared between PoseFusionSystem
// (update at fusion rate) and KalmanPredictSystem (predict at render rate).
struct KalmanBankComponent : mec::ColdComponent<KalmanBankComponent> {
    std::array<KeypointKalmanTracker, kNumKeypoints> trackers{};
};

} // namespace mec
