// Integration test: the §9 ECS systems driven by the mock MithAtomas
// scheduler. Covers (a) the consumer fusion+render path end-to-end, (b)
// UserNeighbourTable filtering (§5.1), and (c) the observer broadcast path.

#include "mec/sim/beacon_pack.h"
#include "mec/sim/synthetic_pose.h"
#include "mec/systems/keypoint_aggregator_system.h"
#include "mec/systems/kalman_predict_system.h"
#include "mec/systems/keypoint_broadcast_system.h"
#include "mec/systems/observer_activation_system.h"
#include "mec/systems/pose_fusion_system.h"
#include "mec/systems/render_serialiser_system.h"
#include "../unit/test_util.h"

#include <cstring>
#include <vector>

using namespace mec;

static void test_consumer_pipeline() {
    mith::World world;
    const mith::EntityId self = world.create_entity();
    world.set_local(self);
    world.add<mith::BehaviourStateComponent>(self);

    KeypointAggregatorSystem aggregator;
    PoseFusionSystem         fusion;
    KalmanPredictSystem      predict;
    RenderSerialiserSystem   render(nullptr); // no live server in the test

    mith::SystemScheduler sched;
    sched.add(&aggregator);
    sched.add(&fusion,  {"KeypointAggregatorSystem"});
    sched.add(&predict, {"PoseFusionSystem"});
    sched.add(&render,  {"KalmanPredictSystem"});

    std::vector<sim::SyntheticObserver> observers = {
        sim::SyntheticObserver(11, 0.03f, 0.9f),
        sim::SyntheticObserver(22, 0.04f, 0.8f),
        sim::SyntheticObserver(33, 0.05f, 0.7f),
    };

    const double dt = 1.0 / 240.0;
    const int steps = 480; // 2 seconds
    double sum_err = 0.0, max_err = 0.0;
    int counted = 0;

    for (int s = 0; s < steps; ++s) {
        const double now = s * dt;
        world.set_now(now);
        world.user_neighbours.clear();
        for (size_t i = 0; i < observers.size(); ++i)
            world.user_neighbours.entries.push_back(
                sim::pack_beacon(observers[i].observe(now),
                                 2000u + static_cast<mith::NodeId>(i), now));
        sched.tick(world, dt);

        if (now > 0.6) {
            auto* pose = world.get<PoseStateComponent>(self);
            CHECK(pose && pose->is_valid);
            if (pose && pose->is_valid) {
                auto gt = sim::ground_truth(now);
                for (int k = 0; k < kNumKeypoints; ++k) {
                    const double ex = pose->keypoints[k].wx - gt[k].wx;
                    const double ey = pose->keypoints[k].wy - gt[k].wy;
                    const double ez = pose->keypoints[k].wz - gt[k].wz;
                    const double err = std::sqrt(ex * ex + ey * ey + ez * ez);
                    if (err > max_err) max_err = err;
                    sum_err += err;
                    ++counted;
                }
            }
        }
    }

    const double mean_err = sum_err / counted;
    std::printf("ECS consumer pipeline: mean err = %.4f m, max = %.4f m\n",
                mean_err, max_err);
    CHECK(mean_err < 0.04);
    CHECK(max_err < 0.15);

    auto* pose = world.get<PoseStateComponent>(self);
    CHECK(pose && pose->observer_count == 3);
    CHECK(render.last_json().find("\"type\":\"pose_frame\"") != std::string::npos);
    CHECK(render.last_json().find("\"observer_count\":3") != std::string::npos);
}

static void test_neighbour_filtering() {
    mith::World world;
    const mith::EntityId self = world.create_entity();
    world.set_local(self);
    KeypointAggregatorSystem aggregator;

    sim::SyntheticObserver obs(7, 0.03f, 0.9f);
    const double now = 1.0;
    world.set_now(now);

    // Two valid TRACKING beacons.
    world.user_neighbours.entries.push_back(sim::pack_beacon(obs.observe(now), 1, now));
    world.user_neighbours.entries.push_back(sim::pack_beacon(obs.observe(now), 2, now));
    // One non-LOS sender (dropped, §5.1).
    world.user_neighbours.entries.push_back(
        sim::pack_beacon(obs.observe(now), 3, now, 0, LOSState::OCCLUDED));
    // One stale beacon outside the 150ms window (dropped, §5.1).
    world.user_neighbours.entries.push_back(
        sim::pack_beacon(obs.observe(now), 4, now - 0.5));
    // One with a foreign schema id (dropped, §5.1).
    auto foreign = sim::pack_beacon(obs.observe(now), 5, now);
    foreign.payload[0] = 0x00; foreign.payload[1] = 0x00; // clobber schema_id
    world.user_neighbours.entries.push_back(foreign);

    aggregator.update(world, 0.0);
    auto* agg = world.get<AggregatedObservationsComponent>(self);
    CHECK(agg && agg->observer_count == 2);
}

static void test_observer_broadcast() {
    mith::World world;
    const mith::EntityId self = world.create_entity();
    world.set_local(self);
    world.add<mith::BehaviourStateComponent>(self);
    world.add<mith::PositionComponent>(self, mith::PositionComponent{1.0f, 2.0f, 0.5f});
    world.add<mith::OrientationComponent>(self); // identity

    CameraIntrinsicsComponent cam;
    cam.intrinsics.fx = 500; cam.intrinsics.fy = 500;
    cam.intrinsics.cx = 320; cam.intrinsics.cy = 240;
    cam.intrinsics.image_w = 640; cam.intrinsics.image_h = 480;
    world.add<CameraIntrinsicsComponent>(self, cam);

    // Synthetic local observation: 33 confident keypoints, centred, depth 3m.
    LatestObservationComponent lo;
    lo.obs.keypoint_count = 33;
    lo.obs.frame_id = 42;
    lo.depth_stable = true;
    for (int i = 0; i < 33; ++i) {
        lo.obs.keypoints[i].x = 0.5f;
        lo.obs.keypoints[i].y = 0.5f;
        lo.obs.keypoints[i].depth_hint = 3.0f;
        lo.obs.keypoints[i].confidence = 0.9f;
    }
    world.add<LatestObservationComponent>(self, lo);

    ObserverActivationSystem activation;
    KeypointBroadcastSystem  broadcast;

    // Drive activation past acquire_frames -> TRACKING.
    for (int i = 0; i < 12; ++i) activation.update(world, 0.0);
    auto* los = world.get<LOSStateComponent>(self);
    CHECK(los && los->state == LOSState::TRACKING);

    // Broadcast should now emit a payload on the user beacon channel.
    world.set_now(5.0);
    broadcast.update(world, 0.0);
    CHECK(world.user_beacon.sent);
    CHECK(world.user_beacon.count == 1);

    KeypointFramePayload pl;
    std::memcpy(&pl, world.user_beacon.last_payload.data(), sizeof(pl));
    CHECK(pl.schema_id == kMecSchemaId);
    CHECK(pl.keypoint_count == kNumKeypoints);
    // Centre keypoint at depth 3, identity orientation, +position offset:
    // camera ray (0,0,3) -> world (1, 2, 0.5+3) = (1,2,3.5).
    CHECK_NEAR(unpack_metres(pl.keypoints[0].wx), 1.0, 0.02);
    CHECK_NEAR(unpack_metres(pl.keypoints[0].wy), 2.0, 0.02);
    CHECK_NEAR(unpack_metres(pl.keypoints[0].wz), 3.5, 0.02);
}

int main() {
    test_consumer_pipeline();
    test_neighbour_filtering();
    test_observer_broadcast();
    RUN_TESTS_RETURN();
}
