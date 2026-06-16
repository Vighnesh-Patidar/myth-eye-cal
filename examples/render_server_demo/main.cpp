// render_server_demo - runs the no-camera fusion sim in real time and serves
// fused "pose_frame" JSON over WebSocket (ARCHITECTURE.md §6). Open
// viewer/myth-eye-cal-viewer.html and point it at ws://<host>:<port>/pose.
//
//   args: <port=8080> <num_observers=3> <seconds=0 (0 = run forever)>

#include "mec/fusion/keypoint_kalman.h"
#include "mec/fusion/multi_observer_fusion.h"
#include "mec/render/pose_serialiser.h"
#include "mec/render/websocket_render_server.h"
#include "mec/sim/synthetic_pose.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>

using clk = std::chrono::steady_clock;

int main(int argc, char** argv) {
    const uint16_t port    = (argc > 1) ? static_cast<uint16_t>(std::atoi(argv[1])) : 8080;
    int            n_obs   = (argc > 2) ? std::atoi(argv[2]) : 3;
    const double   seconds = (argc > 3) ? std::atof(argv[3]) : 0.0;
    if (n_obs < 1) n_obs = 1;

    mec::WebSocketRenderServer server;
    if (!server.start(port)) {
        std::fprintf(stderr, "failed to bind port %u\n", port);
        return 1;
    }
    std::fprintf(stderr,
                 "render_server_demo: serving ws://0.0.0.0:%u/pose  "
                 "(%d observers; %s)\n",
                 server.port(), n_obs,
                 seconds > 0 ? "timed" : "Ctrl-C to stop");

    std::vector<mec::sim::SyntheticObserver> observers;
    for (int i = 0; i < n_obs; ++i)
        observers.emplace_back(1000u + i, 0.03f + 0.01f * i, 0.9f - 0.1f * i);
    std::array<mec::KeypointKalmanTracker, mec::kNumKeypoints> trackers;

    const double kFusionDt = 1.0 / 25.0;
    const double kRenderDt = 1.0 / 60.0;
    double next_fusion = 0.0, next_render = 0.0, last_log = 0.0;

    const auto t0 = clk::now();
    for (;;) {
        const double now =
            std::chrono::duration<double>(clk::now() - t0).count();
        if (seconds > 0.0 && now >= seconds) break;

        server.poll_events(1); // accept + handshakes + client control frames

        // Fusion at 25Hz: feed each tracker the fused observation.
        while (next_fusion <= now) {
            std::vector<std::array<mec::WorldKeypoint, mec::kNumKeypoints>> obs;
            for (auto& o : observers) obs.push_back(o.observe(next_fusion));
            for (int k = 0; k < mec::kNumKeypoints; ++k) {
                std::vector<mec::WorldKeypoint> per_kp;
                for (auto& ob : obs) per_kp.push_back(ob[k]);
                mec::WorldKeypoint fused;
                if (mec::MultiObserverFusion::fuse(per_kp, fused))
                    trackers[k].update(fused);
            }
            next_fusion += kFusionDt;
        }

        // Render/broadcast at 60Hz from Kalman predictions.
        if (next_render <= now && trackers[0].is_initialised()) {
            mec::FusedPose pose;
            pose.timestamp_s = now;
            pose.observer_count = static_cast<uint8_t>(observers.size());
            float conf_sum = 0.0f;
            for (int k = 0; k < mec::kNumKeypoints; ++k) {
                pose.keypoints[k] = trackers[k].predict(now);
                conf_sum += pose.keypoints[k].confidence;
            }
            pose.mean_confidence = conf_sum / mec::kNumKeypoints;
            pose.is_valid = true;
            server.broadcast(mec::serialise_pose(pose));
            next_render += kRenderDt;
        }

        if (now - last_log >= 1.0) {
            std::fprintf(stderr, "  clients=%zu t=%.1fs\n",
                         server.client_count(), now);
            last_log = now;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    server.stop();
    return 0;
}
