// sim_pose_demo - simulated multi-observer fusion with no camera.
//
// Spawns N synthetic LOS observers of one moving person, fuses their noisy
// world-frame keypoints (MultiObserverFusion), smooths with per-keypoint
// Kalman trackers, and prints "pose_frame" JSON (§6.3) at the render rate.
//
// This is the v0.1 fusion core running end-to-end on Linux, no MithAtomas.

#include "mec/fusion/keypoint_kalman.h"
#include "mec/fusion/multi_observer_fusion.h"
#include "mec/render/pose_serialiser.h"
#include "mec/sim/synthetic_pose.h"

#include <array>
#include <cstdio>
#include <vector>

int main(int argc, char** argv) {
    int   num_observers = (argc > 1) ? std::atoi(argv[1]) : 3;
    int   frames        = (argc > 2) ? std::atoi(argv[2]) : 30;
    if (num_observers < 1) num_observers = 1;

    // Heterogeneous observers: differing noise / confidence (viewing angle).
    std::vector<mec::sim::SyntheticObserver> observers;
    for (int i = 0; i < num_observers; ++i) {
        const float noise = 0.03f + 0.01f * static_cast<float>(i); // 3cm, 4cm, ...
        const float conf  = 0.9f - 0.1f * static_cast<float>(i);
        observers.emplace_back(1000u + i, noise, conf > 0.3f ? conf : 0.3f);
    }

    std::array<mec::KeypointKalmanTracker, mec::kNumKeypoints> trackers;

    const double fusion_dt = 1.0 / 25.0; // fusion at 25Hz (inference rate)
    const double render_dt = 1.0 / 60.0; // render at 60Hz (Kalman predict)

    std::fprintf(stderr,
                 "sim_pose_demo: %d observers, %d fusion frames @25Hz, render @60Hz\n",
                 num_observers, frames);

    for (int f = 0; f < frames; ++f) {
        const double t = f * fusion_dt;

        // Gather all observers' keypoints for this fusion tick.
        std::vector<std::array<mec::WorldKeypoint, mec::kNumKeypoints>> obs;
        obs.reserve(observers.size());
        for (auto& o : observers) obs.push_back(o.observe(t));

        // Fuse + update Kalman per keypoint.
        for (int k = 0; k < mec::kNumKeypoints; ++k) {
            std::vector<mec::WorldKeypoint> per_kp;
            per_kp.reserve(obs.size());
            for (auto& ob : obs) per_kp.push_back(ob[k]);

            mec::WorldKeypoint fused;
            if (mec::MultiObserverFusion::fuse(per_kp, fused))
                trackers[k].update(fused);
        }

        // Emit a render frame from Kalman predictions (one per fusion tick here;
        // a real renderer would predict at 60Hz between ticks - see render_dt).
        (void)render_dt;
        mec::FusedPose pose;
        pose.timestamp_s = t;
        pose.observer_count = static_cast<uint8_t>(observers.size());
        float conf_sum = 0.0f;
        for (int k = 0; k < mec::kNumKeypoints; ++k) {
            pose.keypoints[k] = trackers[k].predict(t);
            conf_sum += pose.keypoints[k].confidence;
        }
        pose.mean_confidence = conf_sum / mec::kNumKeypoints;
        pose.is_valid = trackers[0].is_initialised();

        std::printf("%s\n", mec::serialise_pose(pose).c_str());
    }
    return 0;
}
