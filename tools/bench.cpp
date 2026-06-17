// Micro-benchmarks + accuracy harness for the fusion core (host-side).
// Produces the numbers quoted in docs/METRICS_REPORT.md. No external deps.
//
//   build/  ->  ./mec_bench
//
// Reports per-call latency for the hot fusion ops, end-to-end fusion-pipeline
// throughput, and fused accuracy vs. synthetic ground truth.

#include "mec/fusion/keypoint_kalman.h"
#include "mec/fusion/multi_observer_fusion.h"
#include "mec/render/pose_serialiser.h"
#include "mec/sim/synthetic_pose.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

using clk = std::chrono::steady_clock;

template <class F>
static double time_ns_per(const char* label, long iters, F&& fn) {
    const auto t0 = clk::now();
    for (long i = 0; i < iters; ++i) fn(i);
    const auto t1 = clk::now();
    const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count() / iters;
    std::printf("  %-34s %8.1f ns/op   (%ld iters)\n", label, ns, iters);
    return ns;
}

int main() {
    using namespace mec;

    std::printf("== fusion core micro-benchmarks ==\n");

    // --- single-keypoint scalar fuse (3 observers) ---
    {
        std::vector<WorldKeypoint> v(3);
        for (int i = 0; i < 3; ++i) {
            v[i].wx = 0.1f * i; v[i].wy = 0.2f; v[i].wz = 3.0f;
            v[i].uncertainty_r = 0.03f + 0.01f * i; v[i].confidence = 0.9f;
        }
        volatile float sink = 0;
        time_ns_per("fuse() scalar, 3 obs", 2000000, [&](long) {
            WorldKeypoint o; MultiObserverFusion::fuse(v, o); sink += o.wx;
        });
        (void)sink;
    }

    // --- single-keypoint anisotropic fuse (3 orthogonal rays) ---
    {
        std::vector<WorldKeypoint> v(3);
        const float rays[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        for (int i = 0; i < 3; ++i) {
            v[i].wx = 0.0f; v[i].wy = 0.0f; v[i].wz = 0.0f;
            v[i].rx = rays[i][0]; v[i].ry = rays[i][1]; v[i].rz = rays[i][2];
            v[i].uncertainty_r = 0.02f; v[i].depth_uncertainty = 0.5f; v[i].confidence = 0.9f;
        }
        volatile float sink = 0;
        time_ns_per("fuse_anisotropic(), 3 obs", 2000000, [&](long) {
            WorldKeypoint o; MultiObserverFusion::fuse_anisotropic(v, o); sink += o.wx;
        });
        (void)sink;
    }

    // --- Kalman update + predict ---
    {
        KeypointKalmanTracker t;
        WorldKeypoint m; m.uncertainty_r = 0.03f; m.confidence = 0.9f;
        volatile float sink = 0;
        time_ns_per("Kalman update()", 2000000, [&](long i) {
            m.wx = 0.001f * i; m.timestamp_s = 0.04 * i; t.update(m);
        });
        KeypointKalmanTracker t2; m.timestamp_s = 0; t2.update(m);
        time_ns_per("Kalman predict()", 2000000, [&](long i) {
            WorldKeypoint p = t2.predict(0.04 * i); sink += p.wx;
        });
        (void)sink;
    }

    // --- pose_frame serialise (17 keypoints) ---
    {
        FusedPose pose; pose.observer_count = 3; pose.is_valid = true;
        for (int k = 0; k < kNumKeypoints; ++k) {
            pose.keypoints[k].id = (uint8_t)k; pose.keypoints[k].wx = 0.1f * k;
            pose.keypoints[k].wz = 1.0f + k; pose.keypoints[k].confidence = 0.8f;
        }
        volatile size_t sink = 0;
        time_ns_per("serialise_pose() 17 kp", 500000, [&](long i) {
            pose.timestamp_s = 0.001 * i; sink += serialise_pose(pose).size();
        });
        (void)sink;
    }

    // --- end-to-end fusion pipeline throughput + accuracy ---
    std::printf("\n== end-to-end fusion pipeline (3 observers, 17 kp) ==\n");
    {
        std::vector<sim::SyntheticObserver> observers = {
            sim::SyntheticObserver(11, 0.03f, 0.9f),
            sim::SyntheticObserver(22, 0.04f, 0.8f),
            sim::SyntheticObserver(33, 0.05f, 0.7f),
        };
        std::array<KeypointKalmanTracker, kNumKeypoints> trackers;

        const double fusion_dt = 1.0 / 25.0;
        const long frames = 200000;
        double sum_err = 0, max_err = 0; long counted = 0;

        const auto t0 = clk::now();
        for (long f = 0; f < frames; ++f) {
            const double now = f * fusion_dt;
            std::array<std::array<WorldKeypoint, kNumKeypoints>, 3> obs;
            for (int o = 0; o < 3; ++o) obs[o] = observers[o].observe(now);
            for (int k = 0; k < kNumKeypoints; ++k) {
                std::vector<WorldKeypoint> per_kp = {obs[0][k], obs[1][k], obs[2][k]};
                WorldKeypoint fused;
                if (MultiObserverFusion::fuse(per_kp, fused)) trackers[k].update(fused);
            }
            if (f > 50) {
                auto gt = sim::ground_truth(now);
                for (int k = 0; k < kNumKeypoints; ++k) {
                    WorldKeypoint p = trackers[k].predict(now);
                    const double dx = p.wx - gt[k].wx, dy = p.wy - gt[k].wy, dz = p.wz - gt[k].wz;
                    const double err = std::sqrt(dx * dx + dy * dy + dz * dz);
                    sum_err += err; if (err > max_err) max_err = err; ++counted;
                }
            }
        }
        const auto t1 = clk::now();
        const double total_s = std::chrono::duration<double>(t1 - t0).count();
        const double per_frame_us = total_s / frames * 1e6;
        std::printf("  per-frame fuse+update+predict     %8.2f us/frame\n", per_frame_us);
        std::printf("  sustained throughput              %8.0f frames/s\n", frames / total_s);
        std::printf("  fused accuracy mean err           %8.4f m\n", sum_err / counted);
        std::printf("  fused accuracy max err            %8.4f m\n", max_err);
    }

    return 0;
}
