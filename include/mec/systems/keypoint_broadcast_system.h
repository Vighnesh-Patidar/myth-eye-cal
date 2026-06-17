#pragma once

// KeypointBroadcastSystem (§9, 20-30Hz). When the node is TRACKING, projects
// its latest pose observation to the world frame, downsamples 33->17, packs a
// KeypointFramePayload, and broadcasts it on the user beacon channel (§4.5).

#include "mec/components/camera_intrinsics_component.h"
#include "mec/components/internal_components.h"
#include "mec/components/los_state_component.h"
#include "mec/components/observer_metrics_component.h"
#include "mec/observer/keypoint_projector.h"
#include "mec/ecs/world.h"

#include <array>
#include <cstring>

namespace mec {

class KeypointBroadcastSystem : public mec::System {
public:
    const char* name() const override { return "KeypointBroadcastSystem"; }
    double      rate_hz() const override { return 25.0; }

    void update(mec::World& w, double) override {
        const mec::EntityId e = w.local();
        auto* los = w.get<LOSStateComponent>(e);
        if (!los || los->state != LOSState::TRACKING) return; // idle unless LOS

        auto* obsc = w.get<LatestObservationComponent>(e);
        auto* cam  = w.get<CameraIntrinsicsComponent>(e);
        auto* pos  = w.get<mec::PositionComponent>(e);
        auto* ori  = w.get<mec::OrientationComponent>(e);
        if (!obsc || !cam || !pos || !ori) return;

        KeypointProjector proj;
        proj.set_camera_to_body(cam->cam_to_body_rot, cam->cam_to_body_trans);
        NodePose np;
        np.position    = Vec3{pos->x, pos->y, pos->z};
        np.orientation = Quat{ori->qw, ori->qx, ori->qy, ori->qz};

        KeypointFramePayload pl;
        pl.keypoint_count = kNumKeypoints;
        pl.frame_id    = static_cast<uint16_t>(obsc->obs.frame_id);
        pl.timestamp_ms = static_cast<uint32_t>(w.now_s() * 1000.0);
        for (int i = 0; i < kNumKeypoints; ++i) {
            Keypoint kp = obsc->obs.keypoints[kMediapipe33To17[i]];
            kp.id = static_cast<uint8_t>(i);
            const WorldKeypoint wk = proj.project(kp, cam->intrinsics, np, w.now_s());
            pl.keypoints[i].wx = pack_metres(wk.wx);
            pl.keypoints[i].wy = pack_metres(wk.wy);
            pl.keypoints[i].wz = pack_metres(wk.wz);
            pl.keypoints[i].confidence = pack_confidence(wk.confidence);
        }

        std::array<uint8_t, 128> bytes{};
        std::memcpy(bytes.data(), &pl, sizeof(pl));
        w.user_beacon.broadcast(bytes);

        auto& m = w.get_or_add<ObserverMetricsComponent>(e);
        m.fps = static_cast<float>(rate_hz());
    }
};

} // namespace mec
