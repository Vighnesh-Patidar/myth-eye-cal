#pragma once

// RenderSerialiserSystem (§9, 60Hz). Serialises PoseStateComponent into the
// §6.3 "pose_frame" JSON and broadcasts it to connected browsers via the
// WebSocket render server. The server pointer may be null (e.g. in tests), in
// which case the latest JSON is just retained for inspection.

#include "mec/components/pose_state_component.h"
#include "mec/render/pose_serialiser.h"
#include "mec/render/websocket_render_server.h"
#include "mec/ecs/world.h"

#include <string>

namespace mec {

class RenderSerialiserSystem : public mec::System {
public:
    explicit RenderSerialiserSystem(WebSocketRenderServer* server = nullptr)
        : server_(server) {}

    const char* name() const override { return "RenderSerialiserSystem"; }
    double      rate_hz() const override { return 60.0; }

    void update(mec::World& w, double) override {
        auto* pose = w.get<PoseStateComponent>(w.local());
        if (!pose || !pose->is_valid) return;

        FusedPose fp;
        fp.timestamp_s     = pose->last_update_s;
        fp.observer_count  = pose->observer_count;
        fp.mean_confidence = pose->mean_confidence;
        fp.is_valid        = true;
        for (int k = 0; k < kNumKeypoints; ++k) fp.keypoints[k] = pose->keypoints[k];

        last_json_ = serialise_pose(fp);
        if (server_) {
            server_->poll_events(0);
            server_->broadcast(last_json_);
        }
    }

    const std::string& last_json() const { return last_json_; }

private:
    WebSocketRenderServer* server_;
    std::string           last_json_;
};

} // namespace mec
