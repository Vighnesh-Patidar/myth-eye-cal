#include "mec/observer/temporal_stereo_depth.h"
#include "test_util.h"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace mec;

static float pattern(float x, float y) {
    float v = 128.0f + 50.0f * std::sin(0.20f * x)
                     + 40.0f * std::cos(0.13f * y)
                     + 30.0f * std::sin(0.07f * (x + y));
    return std::clamp(v, 0.0f, 255.0f);
}
static void fill(std::vector<uint8_t>& img, int w, int h, float dx, float dy) {
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            img[y * w + x] = static_cast<uint8_t>(pattern(x - dx, y - dy) + 0.5f);
}

int main() {
    // --- kalman_fuse ---
    CHECK_NEAR(TemporalStereoDepth::kalman_fuse(3.0f, 3.0f, 0.2f, 0.05f), 3.0, 1e-4);
    {   // imu (tight sigma) dominates over the lens prior
        const float f = TemporalStereoDepth::kalman_fuse(2.0f, 3.0f, 0.2f, 0.05f);
        CHECK(f > 2.9f && f < 3.0f);
    }

    const int w = 160, h = 120;
    std::vector<uint8_t> curr(w * h), prev(w * h);
    fill(curr, w, h, 0.0f, 0.0f);
    const float disp = 4.0f;
    fill(prev, w, h, disp, 0.0f);             // lateral camera translation
    const Frame pf{prev.data(), w, h, 0};
    const Frame cf{curr.data(), w, h, 1000000};

    TemporalStereoDepth tsd(w, h);

    // Baseline below 5mm -> lens prior only.
    {
        PoseObservation o; o.keypoint_count = 3;
        for (int i = 0; i < 3; ++i) {
            o.keypoints[i].x = 0.5f; o.keypoints[i].y = 0.5f;
            o.keypoints[i].depth_hint = 99.0f;
        }
        IMUFrame imu; imu.baseline_m = 0.002f;
        tsd.resolve(o, pf, cf, imu, 500.0f, 2.5f);
        for (int i = 0; i < 3; ++i) CHECK_NEAR(o.keypoints[i].depth_hint, 2.5, 1e-4);
    }

    // Baseline above 500mm -> frame skipped, depth_hint untouched.
    {
        PoseObservation o; o.keypoint_count = 1;
        o.keypoints[0].x = 0.5f; o.keypoints[0].y = 0.5f; o.keypoints[0].depth_hint = 42.0f;
        IMUFrame imu; imu.baseline_m = 0.7f;
        tsd.resolve(o, pf, cf, imu, 500.0f, 2.5f);
        CHECK_NEAR(o.keypoints[0].depth_hint, 42.0, 1e-4);
    }

    // Normal case: recovered depth ~ focal * baseline / disparity, fused.
    {
        const float f = 500.0f, b = 0.05f, lens = 10.0f;
        const float px[3] = {80, 60, 100}, py[3] = {60, 50, 70};
        PoseObservation o; o.keypoint_count = 3;
        for (int i = 0; i < 3; ++i) {
            o.keypoints[i].x = px[i] / w; o.keypoints[i].y = py[i] / h;
        }
        IMUFrame imu; imu.baseline_m = b;
        tsd.resolve(o, pf, cf, imu, f, lens);

        const float imu_depth = f * b / disp; // 6.25 m
        const float expected =
            TemporalStereoDepth::kalman_fuse(lens, imu_depth, 0.2f, 0.05f);
        for (int i = 0; i < 3; ++i) {
            CHECK(o.keypoints[i].depth_hint > 0.0f);
            CHECK_NEAR(o.keypoints[i].depth_hint, expected, 0.6);
        }
    }

    RUN_TESTS_RETURN();
}
