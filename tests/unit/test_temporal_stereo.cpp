#include "mec/math.h"
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

// Plain content shift: img(p) = pattern(p - d).
static void fill(std::vector<uint8_t>& img, int w, int h, float dx, float dy) {
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x)
            img[y * w + x] = static_cast<uint8_t>(pattern(x - dx, y - dy) + 0.5f);
}

// Build prev as the current pattern warped by the infinite homography for
// camera rotation Rpc (curr->prev directions) plus a lateral pixel translation
// (tx,ty): prev[q] = pattern( pi( Rpc^-1 · K^-1 (q - t) ) ). Tracking curr->prev
// then yields flow = rotational_flow + (tx,ty) at every keypoint.
static void warp(std::vector<uint8_t>& prev, int w, int h,
                 const CameraIntrinsics& K, const Quat& Rpc, float tx, float ty) {
    const Quat Rinv = conj(Rpc);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const float qx = x - tx, qy = y - ty;
            const Vec3 ray{(qx - K.cx) / K.fx, (qy - K.cy) / K.fy, 1.0f};
            const Vec3 r = rotate(Rinv, ray);
            float sx = qx, sy = qy;
            if (r.z > 1e-3f) { sx = K.fx * r.x / r.z + K.cx; sy = K.fy * r.y / r.z + K.cy; }
            prev[y * w + x] = static_cast<uint8_t>(pattern(sx, sy) + 0.5f);
        }
    }
}

static PoseObservation kps_at(const float (*px)[2], int n, int w, int h) {
    PoseObservation o; o.keypoint_count = static_cast<uint8_t>(n);
    for (int i = 0; i < n; ++i) { o.keypoints[i].x = px[i][0] / w; o.keypoints[i].y = px[i][1] / h; }
    return o;
}

int main() {
    const int w = 160, h = 120;
    CameraIntrinsics K; K.fx = 500; K.fy = 500; K.cx = 80; K.cy = 60;
    K.image_w = w; K.image_h = h;
    const float kPts[3][2] = {{80, 60}, {60, 50}, {100, 70}};

    // --- kalman_fuse ---
    CHECK_NEAR(TemporalStereoDepth::kalman_fuse(3.0f, 3.0f, 0.2f, 0.05f), 3.0, 1e-4);
    {
        const float f = TemporalStereoDepth::kalman_fuse(2.0f, 3.0f, 0.2f, 0.05f);
        CHECK(f > 2.9f && f < 3.0f);
    }

    std::vector<uint8_t> curr(w * h), prev(w * h);
    fill(curr, w, h, 0.0f, 0.0f);
    const Frame cf{curr.data(), w, h, 1000000};

    TemporalStereoDepth tsd(w, h);

    // Baseline below 5mm -> lens prior only.
    {
        fill(prev, w, h, 4.0f, 0.0f);
        const Frame pf{prev.data(), w, h, 0};
        PoseObservation o = kps_at(kPts, 3, w, h);
        for (int i = 0; i < 3; ++i) o.keypoints[i].depth_hint = 99.0f;
        IMUFrame imu; imu.baseline_m = 0.002f;
        tsd.resolve(o, pf, cf, imu, K, 2.5f);
        for (int i = 0; i < 3; ++i) CHECK_NEAR(o.keypoints[i].depth_hint, 2.5, 1e-4);
    }

    // Baseline above 500mm -> frame skipped, depth_hint untouched.
    {
        fill(prev, w, h, 4.0f, 0.0f);
        const Frame pf{prev.data(), w, h, 0};
        PoseObservation o = kps_at(kPts, 1, w, h);
        o.keypoints[0].depth_hint = 42.0f;
        IMUFrame imu; imu.baseline_m = 0.7f;
        tsd.resolve(o, pf, cf, imu, K, 2.5f);
        CHECK_NEAR(o.keypoints[0].depth_hint, 42.0, 1e-4);
    }

    // Pure translation (no rotation): depth = focal * baseline / disparity.
    {
        const float b = 0.05f, disp = 4.0f, lens = 10.0f;
        fill(prev, w, h, disp, 0.0f);
        const Frame pf{prev.data(), w, h, 0};
        PoseObservation o = kps_at(kPts, 3, w, h);
        IMUFrame imu; imu.baseline_m = b; // dq defaults to identity
        tsd.resolve(o, pf, cf, imu, K, lens);
        const float expected =
            TemporalStereoDepth::kalman_fuse(lens, b * K.fx / disp, 0.2f, 0.05f);
        for (int i = 0; i < 3; ++i) CHECK_NEAR(o.keypoints[i].depth_hint, expected, 0.6);
    }

    // --- §15.7 de-rotation ---
    // Yaw about camera Y by theta -> ~ focal*theta px of rotational flow.
    const float theta = 0.01f;
    const Quat Rpc{std::cos(theta / 2), 0.0f, std::sin(theta / 2), 0.0f};

    // Pure rotation, no translation: de-rotation cancels the flow, leaving
    // sub-threshold disparity -> falls back to the lens prior (NOT a bogus
    // shallow depth from the ~5px rotational flow).
    {
        warp(prev, w, h, K, Rpc, 0.0f, 0.0f);
        const Frame pf{prev.data(), w, h, 0};
        PoseObservation o = kps_at(kPts, 3, w, h);
        IMUFrame imu; imu.baseline_m = 0.05f;
        imu.dqw = Rpc.w; imu.dqx = Rpc.x; imu.dqy = Rpc.y; imu.dqz = Rpc.z;
        tsd.resolve(o, pf, cf, imu, K, 3.3f);
        for (int i = 0; i < 3; ++i) CHECK_NEAR(o.keypoints[i].depth_hint, 3.3, 1e-3);
    }

    // Rotation + translation: de-rotation removes the rotational component and
    // recovers depth from the translational disparity alone.
    {
        const float b = 0.05f, tdisp = 4.0f, lens = 10.0f;
        warp(prev, w, h, K, Rpc, tdisp, 0.0f);
        const Frame pf{prev.data(), w, h, 0};
        PoseObservation o = kps_at(kPts, 3, w, h);
        IMUFrame imu; imu.baseline_m = b;
        imu.dqw = Rpc.w; imu.dqx = Rpc.x; imu.dqy = Rpc.y; imu.dqz = Rpc.z;
        tsd.resolve(o, pf, cf, imu, K, lens);
        const float expected =
            TemporalStereoDepth::kalman_fuse(lens, b * K.fx / tdisp, 0.2f, 0.05f);
        for (int i = 0; i < 3; ++i) CHECK_NEAR(o.keypoints[i].depth_hint, expected, 0.6);
    }

    RUN_TESTS_RETURN();
}
