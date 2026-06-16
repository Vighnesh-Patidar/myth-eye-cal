#pragma once

// Synthetic pose generation for the v0.1 "no camera" path (ARCHITECTURE.md
// §12 roadmap: "Simulated LOS node"). Produces a moving ground-truth skeleton
// and per-observer noisy world-frame observations of it.

#include "mec/types.h"
#include <array>
#include <cmath>
#include <random>

namespace mec::sim {

// Canonical standing skeleton in metres (person facing +X, up is +Z),
// 17 keypoints in the §6.2 order. Origin at the feet midpoint.
inline std::array<WorldKeypoint, kNumKeypoints> base_skeleton() {
    const float coords[kNumKeypoints][3] = {
        {0.00f, 0.00f, 1.70f}, // 0  nose
        {0.03f, 0.00f, 1.73f}, // 1  left eye
        {-0.03f, 0.00f, 1.73f},// 2  right eye
        {0.07f, 0.00f, 1.72f}, // 3  left ear
        {-0.07f, 0.00f, 1.72f},// 4  right ear
        {0.18f, 0.00f, 1.45f}, // 5  left shoulder
        {-0.18f, 0.00f, 1.45f},// 6  right shoulder
        {0.20f, 0.00f, 1.15f}, // 7  left elbow
        {-0.20f, 0.00f, 1.15f},// 8  right elbow
        {0.22f, 0.00f, 0.90f}, // 9  left wrist
        {-0.22f, 0.00f, 0.90f},// 10 right wrist
        {0.12f, 0.00f, 0.95f}, // 11 left hip
        {-0.12f, 0.00f, 0.95f},// 12 right hip
        {0.13f, 0.00f, 0.52f}, // 13 left knee
        {-0.13f, 0.00f, 0.52f},// 14 right knee
        {0.13f, 0.00f, 0.05f}, // 15 left ankle
        {-0.13f, 0.00f, 0.05f},// 16 ankle
    };
    std::array<WorldKeypoint, kNumKeypoints> s{};
    for (int i = 0; i < kNumKeypoints; ++i) {
        s[i].id = static_cast<uint8_t>(i);
        s[i].wx = coords[i][0];
        s[i].wy = coords[i][1];
        s[i].wz = coords[i][2];
        s[i].confidence = 1.0f;
    }
    return s;
}

// Ground-truth pose at time t: the base skeleton walking along +Y with a
// gentle vertical bob. Deterministic, no observation noise.
inline std::array<WorldKeypoint, kNumKeypoints> ground_truth(double t) {
    auto s = base_skeleton();
    const float dy = 0.5f * static_cast<float>(t);              // walk
    const float bob = 0.03f * std::sin(static_cast<float>(t) * 4.0f);
    for (auto& k : s) {
        k.wy += dy;
        k.wz += bob;
        k.timestamp_s = t;
    }
    return s;
}

// One simulated LOS observer: adds Gaussian noise to the ground truth and
// assigns a per-observer confidence / uncertainty. Deterministic per seed.
class SyntheticObserver {
public:
    SyntheticObserver(uint32_t seed, float noise_std_m, float confidence)
        : rng_(seed), noise_(0.0f, noise_std_m),
          noise_std_(noise_std_m), confidence_(confidence) {}

    std::array<WorldKeypoint, kNumKeypoints> observe(double t) {
        auto gt = ground_truth(t);
        for (auto& k : gt) {
            k.wx += noise_(rng_);
            k.wy += noise_(rng_);
            k.wz += noise_(rng_);
            k.confidence = confidence_;
            // Report uncertainty consistent with the injected noise so the
            // fuser weights observers correctly (better observers -> lower r).
            k.uncertainty_r = noise_std_;
            k.timestamp_s = t;
        }
        return gt;
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<float> noise_;
    float noise_std_;
    float confidence_;
};

} // namespace mec::sim
