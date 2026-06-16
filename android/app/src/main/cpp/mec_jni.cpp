// JNI bridge: drives the tested mec_core pipeline from the Android shell.
//
// On-device data flow (single device, v0.3 milestone):
//   Camera2 (Kotlin) -> MediaPipe pose (Kotlin) -> nativeOnFrame
//   SensorManager (Kotlin) 200Hz -> nativeOnImuSample -> IMUIntegrator
//   nativeOnFrame: temporal-stereo depth -> world projection -> feed as one
//   observer -> fusion + Kalman -> WebSocket render (browser).
// Multi-device fusion needs the real mith-atomas transport (mock here, §15.6).

#include "mec/components/pose_state_component.h"
#include "mec/observer/imu_integrator.h"
#include "mec/observer/keypoint_projector.h"
#include "mec/observer/temporal_stereo_depth.h"
#include "mec/render/websocket_render_server.h"
#include "mec/sim/beacon_pack.h"
#include "mec/systems/kalman_predict_system.h"
#include "mec/systems/keypoint_aggregator_system.h"
#include "mec/systems/pose_fusion_system.h"
#include "mec/systems/render_serialiser_system.h"
#include "mith/atomas.h"

#include <android/log.h>
#include <jni.h>
#include <array>
#include <vector>

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "mec_jni", __VA_ARGS__)

namespace {

struct Context {
    int width, height;
    mith::World world;
    mith::EntityId self = 0;
    mith::SystemScheduler sched;
    mec::KeypointAggregatorSystem aggregator;
    mec::PoseFusionSystem fusion;
    mec::KalmanPredictSystem predict;
    mec::WebSocketRenderServer server;
    mec::RenderSerialiserSystem render;
    mec::TemporalStereoDepth depth;
    mec::IMUIntegrator imu;
    mec::KeypointProjector projector;
    mec::CameraIntrinsics intr;
    std::vector<uint8_t> prev_y, cur_y;
    bool have_prev = false;
    uint32_t frame_id = 0;
    double last_ts_s = 0.0;

    Context(int w, int h) : width(w), height(h), render(&server), depth(w, h) {}
};

inline Context* ctx(jlong h) { return reinterpret_cast<Context*>(h); }

} // namespace

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_mythcal_mec_MecNative_nativeInit(JNIEnv*, jobject, jint width, jint height, jint port) {
    Context* c = new Context(width, height);
    c->self = c->world.create_entity();
    c->world.set_local(c->self);
    c->world.add<mith::BehaviourStateComponent>(c->self);

    c->sched.add(&c->aggregator);
    c->sched.add(&c->fusion, {"KeypointAggregatorSystem"});
    c->sched.add(&c->predict, {"PoseFusionSystem"});
    c->sched.add(&c->render, {"KalmanPredictSystem"});

    if (!c->server.start(static_cast<uint16_t>(port))) {
        LOGI("WebSocket server failed to bind port %d", port);
    } else {
        LOGI("WebSocket render server on ws://0.0.0.0:%d/pose", c->server.port());
    }
    return reinterpret_cast<jlong>(c);
}

JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeSetIntrinsics(JNIEnv*, jobject, jlong h,
                                                   jfloat fx, jfloat fy, jfloat cx, jfloat cy) {
    Context* c = ctx(h);
    c->intr.fx = fx; c->intr.fy = fy; c->intr.cx = cx; c->intr.cy = cy;
    c->intr.image_w = c->width; c->intr.image_h = c->height;
}

JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeOnImuSample(JNIEnv*, jobject, jlong h,
                                                 jfloat ax, jfloat ay, jfloat az,
                                                 jfloat gx, jfloat gy, jfloat gz, jfloat dt) {
    ctx(h)->imu.integrate(mec::Vec3{ax, ay, az}, mec::Vec3{gx, gy, gz}, dt);
}

JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativePoll(JNIEnv*, jobject, jlong h) {
    ctx(h)->server.poll_events(0);
}

JNIEXPORT jint JNICALL
Java_com_mythcal_mec_MecNative_nativeServerPort(JNIEnv*, jobject, jlong h) {
    return ctx(h)->server.port();
}

// landmarks: float[count*4] = (x_norm, y_norm, z, visibility) per MediaPipe.
JNIEXPORT jstring JNICALL
Java_com_mythcal_mec_MecNative_nativeOnFrame(JNIEnv* env, jobject, jlong h,
                                             jbyteArray y_plane, jint w, jint hgt, jlong ts_ns,
                                             jfloatArray landmarks, jint count, jfloat lens_prior_m) {
    Context* c = ctx(h);
    const double ts = static_cast<double>(ts_ns) * 1e-9;

    // Copy the Y plane into the current buffer.
    const jsize n = env->GetArrayLength(y_plane);
    c->cur_y.resize(static_cast<size_t>(n));
    env->GetByteArrayRegion(y_plane, 0, n, reinterpret_cast<jbyte*>(c->cur_y.data()));

    // Build the pose observation from MediaPipe landmarks.
    mec::PoseObservation obs;
    const int kc = (count < 33) ? count : 33;
    obs.keypoint_count = static_cast<uint8_t>(kc);
    obs.frame_id = ++c->frame_id;
    obs.timestamp_s = ts;
    jfloat* lm = env->GetFloatArrayElements(landmarks, nullptr);
    for (int i = 0; i < kc; ++i) {
        obs.keypoints[i].id = static_cast<uint8_t>(i);
        obs.keypoints[i].x = lm[i * 4 + 0];
        obs.keypoints[i].y = lm[i * 4 + 1];
        obs.keypoints[i].confidence = lm[i * 4 + 3];
    }
    env->ReleaseFloatArrayElements(landmarks, lm, JNI_ABORT);

    // Temporal-stereo depth (needs a previous frame).
    const mec::IMUFrame imf = c->imu.consume(ts);
    if (c->have_prev) {
        const mec::Frame prev{c->prev_y.data(), w, hgt, ts_ns};
        const mec::Frame cur{c->cur_y.data(), w, hgt, ts_ns};
        c->depth.resolve(obs, prev, cur, imf, c->intr, lens_prior_m);
    } else {
        for (int i = 0; i < kc; ++i) obs.keypoints[i].depth_hint = lens_prior_m;
    }

    // Project to the world frame (node pinned at origin; orientation from IMU).
    mec::NodePose np;
    np.position = mec::Vec3{0.0f, 0.0f, 0.0f};
    np.orientation = c->imu.orientation();
    std::array<mec::WorldKeypoint, mec::kNumKeypoints> kps{};
    for (int i = 0; i < mec::kNumKeypoints; ++i) {
        mec::Keypoint kp = obs.keypoints[mec::kMediapipe33To17[i]];
        kp.id = static_cast<uint8_t>(i);
        kps[i] = c->projector.project(kp, c->intr, np, ts);
    }

    // Feed as a single LOS observer, then run the fusion+render pipeline.
    c->world.user_neighbours.clear();
    c->world.user_neighbours.entries.push_back(
        mec::sim::pack_beacon(kps, 1, ts, static_cast<uint16_t>(c->frame_id),
                              mec::LOSState::TRACKING, np.position));
    c->world.set_now(ts);
    c->server.poll_events(0);
    double dt = ts - c->last_ts_s;
    if (dt <= 0.0 || dt > 1.0) dt = 1.0 / 30.0;
    c->sched.tick(c->world, dt);
    c->last_ts_s = ts;

    std::swap(c->prev_y, c->cur_y);
    c->have_prev = true;

    return env->NewStringUTF(c->render.last_json().c_str());
}

JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeShutdown(JNIEnv*, jobject, jlong h) {
    Context* c = ctx(h);
    if (!c) return;
    c->server.stop();
    delete c;
}

} // extern "C"
