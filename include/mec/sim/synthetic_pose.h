#pragma once

// Synthetic pose generation for the v0.1 "no camera" path (ARCHITECTURE.md
// §12 roadmap: "Simulated LOS node"). Produces a moving ground-truth skeleton
// and per-observer noisy world-frame observations of it.

#include "mec/math.h"
#include "mec/types.h"
#include <algorithm>
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
    // Isotropic observer: equal Gaussian noise on all axes.
    SyntheticObserver(uint32_t seed, float noise_std_m, float confidence)
        : rng_(seed), noise_(0.0f, noise_std_m),
          noise_std_(noise_std_m), confidence_(confidence) {}

    // Anisotropic observer (§15.3): viewed from `viewpoint`, with small lateral
    // noise and large along-ray (depth) noise — the monocular error model.
    SyntheticObserver(uint32_t seed, const Vec3& viewpoint,
                      float sigma_lat, float sigma_depth, float confidence)
        : rng_(seed), noise_(0.0f, sigma_lat), noise_std_(sigma_lat),
          confidence_(confidence), viewpoint_(viewpoint),
          sigma_lat_(sigma_lat), sigma_depth_(sigma_depth), anisotropic_(true) {}

    Vec3 viewpoint() const { return viewpoint_; }

    std::array<WorldKeypoint, kNumKeypoints> observe(double t) {
        auto gt = ground_truth(t);
        for (auto& k : gt) {
            if (!anisotropic_) {
                k.wx += noise_(rng_);
                k.wy += noise_(rng_);
                k.wz += noise_(rng_);
                k.confidence = confidence_;
                k.uncertainty_r = noise_std_;
                k.timestamp_s = t;
                continue;
            }
            // Ray from the camera to the keypoint, and a basis perpendicular to it.
            Vec3 ray{k.wx - viewpoint_.x, k.wy - viewpoint_.y, k.wz - viewpoint_.z};
            const float rn = std::max(norm(ray), 1e-6f);
            ray = ray * (1.0f / rn);
            const Vec3 helper = (std::fabs(ray.x) < 0.9f) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
            Vec3 e1 = cross(ray, helper);
            e1 = e1 * (1.0f / std::max(norm(e1), 1e-6f));
            const Vec3 e2 = cross(ray, e1);
            // Anisotropic noise: large along the ray, small laterally.
            const Vec3 n = ray * (sigma_depth_ * unit_(rng_)) +
                           e1 * (sigma_lat_ * unit_(rng_)) +
                           e2 * (sigma_lat_ * unit_(rng_));
            k.wx += n.x; k.wy += n.y; k.wz += n.z;
            k.rx = ray.x; k.ry = ray.y; k.rz = ray.z;
            k.uncertainty_r = sigma_lat_;
            k.depth_uncertainty = sigma_depth_;
            k.confidence = confidence_;
            k.timestamp_s = t;
        }
        return gt;
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<float> noise_;
    std::normal_distribution<float> unit_{0.0f, 1.0f};
    float noise_std_;
    float confidence_;
    Vec3  viewpoint_{};
    float sigma_lat_ = 0.0f;
    float sigma_depth_ = 0.0f;
    bool  anisotropic_ = false;
};

} // namespace mec::sim
