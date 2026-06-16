// ecs_pipeline_demo - the full §5/§6 fusion + render pipeline running through
// the mock MithAtomas ECS (§9 systems + SystemScheduler), serving fused poses
// over WebSocket. The local node is a non-LOS consumer; three synthetic LOS
// neighbours feed observations into the UserNeighbourTable.
//
//   args: <port=8080> <num_observers=3> <seconds=0 (0 = run forever)>

#include "mec/sim/beacon_pack.h"
#include "mec/sim/synthetic_pose.h"
#include "mec/systems/keypoint_aggregator_system.h"
#include "mec/systems/kalman_predict_system.h"
#include "mec/systems/pose_fusion_system.h"
#include "mec/systems/render_serialiser_system.h"
#include "mith/atomas.h"

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
    if (!server.start(port)) { std::fprintf(stderr, "bind failed on %u\n", port); return 1; }
    std::fprintf(stderr, "ecs_pipeline_demo: ws://0.0.0.0:%u/pose  (%d LOS neighbours)\n",
                 server.port(), n_obs);

    mith::World world;
    const mith::EntityId self = world.create_entity();
    world.set_local(self);
    world.add<mith::BehaviourStateComponent>(self); // local node is a consumer

    // §9 fusion + render systems wired with the dependency order from the doc.
    mec::KeypointAggregatorSystem aggregator;
    mec::PoseFusionSystem         fusion;
    mec::KalmanPredictSystem      predict;
    mec::RenderSerialiserSystem   render(&server);

    mith::SystemScheduler sched;
    sched.add(&aggregator);
    sched.add(&fusion,  {"KeypointAggregatorSystem"});
    sched.add(&predict, {"PoseFusionSystem"});
    sched.add(&render,  {"KalmanPredictSystem"});

    std::vector<mec::sim::SyntheticObserver> observers;
    for (int i = 0; i < n_obs; ++i)
        observers.emplace_back(1000u + i, 0.03f + 0.01f * i, 0.9f - 0.1f * i);

    const auto t0 = clk::now();
    double last = 0.0, last_log = 0.0;
    for (;;) {
        const double now = std::chrono::duration<double>(clk::now() - t0).count();
        if (seconds > 0.0 && now >= seconds) break;
        world.set_now(now);

        // Neighbour beacons arrive: refresh the UserNeighbourTable.
        world.user_neighbours.clear();
        for (size_t i = 0; i < observers.size(); ++i)
            world.user_neighbours.entries.push_back(
                mec::sim::pack_beacon(observers[i].observe(now),
                                      2000u + static_cast<mith::NodeId>(i), now));

        server.poll_events(0);
        sched.tick(world, now - last);
        last = now;

        if (now - last_log >= 1.0) {
            auto* pose = world.get<mec::PoseStateComponent>(self);
            std::fprintf(stderr, "  clients=%zu observers=%u meanconf=%.2f t=%.1fs\n",
                         server.client_count(),
                         pose ? pose->observer_count : 0,
                         pose ? pose->mean_confidence : 0.0f, now);
            last_log = now;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    server.stop();
    return 0;
}
