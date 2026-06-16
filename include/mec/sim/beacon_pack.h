#pragma once

// Helper to pack a set of world keypoints into a mith::UserStateVector, as a
// neighbour's user beacon would arrive (§4.5). ECS-side only (depends on the
// mock mith runtime); used by the ECS demo and tests to inject synthetic
// LOS-node observations into the UserNeighbourTable.

#include "mec/math.h"
#include "mec/types.h"
#include "mith/atomas.h"

#include <array>
#include <cstring>

namespace mec::sim {

inline mith::UserStateVector pack_beacon(
    const std::array<WorldKeypoint, kNumKeypoints>& kps,
    mith::NodeId sender, double now, uint16_t frame_id = 0,
    LOSState los = LOSState::TRACKING, Vec3 sender_pos = Vec3{}) {
    KeypointFramePayload pl;
    pl.keypoint_count = kNumKeypoints;
    pl.frame_id = frame_id;
    pl.timestamp_ms = static_cast<uint32_t>(now * 1000.0);
    for (int i = 0; i < kNumKeypoints; ++i) {
        pl.keypoints[i].wx = pack_metres(kps[i].wx);
        pl.keypoints[i].wy = pack_metres(kps[i].wy);
        pl.keypoints[i].wz = pack_metres(kps[i].wz);
        pl.keypoints[i].confidence = pack_confidence(kps[i].confidence);
    }
    mith::UserStateVector usv;
    usv.sender = sender;
    usv.los_state = static_cast<uint8_t>(los);
    usv.recv_time_s = now;
    usv.spx = sender_pos.x; usv.spy = sender_pos.y; usv.spz = sender_pos.z;
    std::memcpy(usv.payload.data(), &pl, sizeof(pl));
    return usv;
}

} // namespace mec::sim
