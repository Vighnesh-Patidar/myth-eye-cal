// Integration test: 3 simulated LOS observers -> fusion -> Kalman -> fused
// pose, checked against ground truth (§12 v0.1: "3 simulated LOS nodes ->
// fused pose"). No camera, no MithAtomas.

#include "mec/fusion/keypoint_kalman.h"
#include "mec/fusion/multi_observer_fusion.h"
#include "mec/render/pose_serialiser.h"
#include "mec/sim/synthetic_pose.h"
#include "../unit/test_util.h"

#include <array>
#include <vector>

using namespace mec;

int main() {
    std::vector<sim::SyntheticObserver> observers = {
        sim::SyntheticObserver(11, 0.03f, 0.9f),
        sim::SyntheticObserver(22, 0.04f, 0.8f),
        sim::SyntheticObserver(33, 0.05f, 0.7f),
    };
    std::array<KeypointKalmanTracker, kNumKeypoints> trackers;

    const double dt = 1.0 / 25.0;
    const int frames = 50;
    double max_err = 0.0;
    double sum_err = 0.0;
    int counted = 0;

    for (int f = 0; f < frames; ++f) {
        const double t = f * dt;
        std::vector<std::array<WorldKeypoint, kNumKeypoints>> obs;
        for (auto& o : observers) obs.push_back(o.observe(t));

        for (int k = 0; k < kNumKeypoints; ++k) {
            std::vector<WorldKeypoint> per_kp;
            for (auto& ob : obs) per_kp.push_back(ob[k]);
            WorldKeypoint fused;
            CHECK(MultiObserverFusion::fuse(per_kp, fused));
            trackers[k].update(fused);
        }

        // After the filter has settled, compare to ground truth.
        if (f > 10) {
            auto gt = sim::ground_truth(t);
            for (int k = 0; k < kNumKeypoints; ++k) {
                WorldKeypoint p = trackers[k].predict(t);
                const double ex = p.wx - gt[k].wx;
                const double ey = p.wy - gt[k].wy;
                const double ez = p.wz - gt[k].wz;
                const double err = std::sqrt(ex * ex + ey * ey + ez * ez);
                if (err > max_err) max_err = err;
                sum_err += err;
                ++counted;
            }
        }
    }

    const double mean_err = sum_err / counted;
    std::printf("fused mean error = %.4f m, max = %.4f m over %d samples\n",
                mean_err, max_err, counted);

    // Fusing 3 noisy observers (3-5cm each) + Kalman smoothing should beat any
    // single observer's noise. Mean error well under 3cm; no wild outliers.
    CHECK(mean_err < 0.03);
    CHECK(max_err < 0.12);

    // Serialiser sanity: produces a non-trivial pose_frame JSON.
    FusedPose pose;
    for (int k = 0; k < kNumKeypoints; ++k) pose.keypoints[k] = trackers[k].predict(frames * dt);
    pose.observer_count = 3;
    const std::string json = serialise_pose(pose);
    CHECK(json.find("\"type\":\"pose_frame\"") != std::string::npos);
    CHECK(json.find("\"observer_count\":3") != std::string::npos);

    RUN_TESTS_RETURN();
}
