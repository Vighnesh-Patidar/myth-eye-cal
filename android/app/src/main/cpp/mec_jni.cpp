// JNI bridge: drives the tested mec_core pipeline from the Android shell.
//
// On-device data flow (single device, v0.3 milestone):
//   Camera2 (Kotlin) -> MediaPipe pose (Kotlin) -> nativeOnFrame
//   SensorManager (Kotlin) 200Hz -> nativeOnImuSample -> IMUIntegrator
//   nativeOnFrame: temporal-stereo depth -> world projection -> feed as one
//   observer -> fusion + Kalman -> WebSocket render (browser).
// Multi-device fusion rides the real mith-atomas transport (MEC_USE_MITH) or
// the UDP stopgap; the local fusion ECS is mec's own runtime (§15.13).

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
#ifdef MEC_USE_MITH
#include "mec/transport/mith_runtime.h"      // real mith-atomas backing
#else
#include "mec/transport/udp_beacon_transport.h" // UDP stopgap
#endif
#include "mec/ecs/world.h"
#include "mec/transport/device_registry.h"

#include <android/log.h>
#include <jni.h>
#include <array>
#include <cstdio>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, "mec_jni", __VA_ARGS__)

namespace {

struct Context {
    int width, height;
    mec::World world;
    mec::EntityId self = 0;
    mec::SystemScheduler sched;
    mec::KeypointAggregatorSystem aggregator;
    mec::PoseFusionSystem fusion;
    mec::KalmanPredictSystem predict;
    mec::WebSocketRenderServer server;
    mec::RenderSerialiserSystem render;
    mec::TemporalStereoDepth depth;
    mec::IMUIntegrator imu;
    mec::KeypointProjector projector;
    mec::CameraIntrinsics intr;
#ifdef MEC_USE_MITH
    mec::MithRuntime transport;              // real mith-atomas (§15.13)
#else
    mec::UdpBeaconTransport transport;       // UDP stopgap (§15.10)
#endif
    mec::Vec3 node_pos{};                     // manual co-localization pin (position)
    mec::Quat node_orientation{};             // absolute orientation (rotation-vector, ENU)
    uint64_t node_id = 0;
    std::vector<uint8_t> prev_y, cur_y;
    bool have_prev = false;
    uint32_t frame_id = 0;
    double last_ts_s = 0.0;
    std::unordered_map<uint64_t, double> neighbor_seen; // sender -> last-heard time
    int last_neighbor_count = 0;
    mec::DeviceRegistry registry;             // discovery + manual connect allowlist
    bool   want_announce = false;             // reply to a received scan probe
    double last_announce_s = 0.0;             // idle presence-announce throttle

    Context(int w, int h) : width(w), height(h), render(&server), depth(w, h) {}
};

inline Context* ctx(jlong h) { return reinterpret_cast<Context*>(h); }

} // namespace

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_mythcal_mec_MecNative_nativeInit(JNIEnv* env, jobject, jint width, jint height,
                                          jint port, jlong node_id, jint beacon_port,
                                          jstring iface_ip) {
    Context* c = new Context(width, height);
    c->node_id = static_cast<uint64_t>(node_id);

    // Wi-Fi interface IPv4 for the beacon transport. On Android the default
    // route is often cellular, so multicast/broadcast must be pinned to the
    // Wi-Fi interface or beacons never reach LAN neighbours. Empty/null = let
    // the kernel choose (0.0.0.0).
    const char* iface_c = iface_ip ? env->GetStringUTFChars(iface_ip, nullptr) : nullptr;
    const char* iface = (iface_c && iface_c[0]) ? iface_c : "0.0.0.0";
    c->self = c->world.create_entity();
    c->world.set_local(c->self);
    c->world.add<mec::BehaviourStateComponent>(c->self);

    c->sched.add(&c->aggregator);
    c->sched.add(&c->fusion, {"KeypointAggregatorSystem"});
    c->sched.add(&c->predict, {"PoseFusionSystem"});
    c->sched.add(&c->render, {"KalmanPredictSystem"});

    if (!c->server.start(static_cast<uint16_t>(port)))
        LOGI("WebSocket server failed to bind port %d", port);
    else
        LOGI("WebSocket render server on ws://0.0.0.0:%d/pose", c->server.port());

#ifdef MEC_USE_MITH
    (void)beacon_port;
    if (c->transport.start(/*swarm_id=*/42, /*group=*/"239.10.20.30", /*port=*/47474, iface))
        LOGI("mith-atomas runtime up (multicast 239.10.20.30:47474 via iface %s)", iface);
    else
        LOGI("mith-atomas runtime failed to start (iface %s)", iface);
#else
    (void)iface;
    if (!c->transport.start(static_cast<uint16_t>(beacon_port), c->node_id))
        LOGI("UDP beacon transport failed to bind port %d", beacon_port);
    else
        LOGI("UDP beacon transport on udp/%d (node %llu)", beacon_port,
             static_cast<unsigned long long>(c->node_id));
#endif
    if (iface_c) env->ReleaseStringUTFChars(iface_ip, iface_c);
    return reinterpret_cast<jlong>(c);
}

JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeSetNodePose(JNIEnv*, jobject, jlong h,
                                                 jfloat x, jfloat y, jfloat z) {
    ctx(h)->node_pos = mec::Vec3{x, y, z};
}

// Absolute device->world orientation from the rotation-vector sensor (gravity +
// magnetic north). Shared reference across phones => aligned world frames, and
// drift-free (§15.12).
JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeSetOrientation(JNIEnv*, jobject, jlong h,
                                                    jfloat qw, jfloat qx, jfloat qy, jfloat qz) {
    ctx(h)->node_orientation = mec::Quat{qw, qx, qy, qz}.normalized();
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

JNIEXPORT jint JNICALL
Java_com_mythcal_mec_MecNative_nativeClientCount(JNIEnv*, jobject, jlong h) {
    return static_cast<jint>(ctx(h)->server.client_count());
}

// Distinct neighbour phones heard over UDP in the last ~2s.
JNIEXPORT jint JNICALL
Java_com_mythcal_mec_MecNative_nativeNeighborCount(JNIEnv*, jobject, jlong h) {
    return ctx(h)->last_neighbor_count;
}

// Observers that contributed to the latest fused pose (local + neighbours).
JNIEXPORT jint JNICALL
Java_com_mythcal_mec_MecNative_nativeObserverCount(JNIEnv*, jobject, jlong h) {
    Context* c = ctx(h);
    auto* pose = c->world.get<mec::PoseStateComponent>(c->self);
    return pose ? static_cast<jint>(pose->observer_count) : 0;
}

// --- Discovery + manual connect control (§ device control) -----------------

// Send a discovery probe ("who's there?"). Peers reply with their presence.
JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeScan(JNIEnv*, jobject, jlong h) {
    ctx(h)->transport.scan();
}

// Snapshot of discovered devices, one per line:
//   "<id>,<age_ms>,<los>,<connected>,<has_data>,<x>,<y>,<z>"
// id is an unsigned-decimal u64; connected/has_data are 0/1.
JNIEXPORT jstring JNICALL
Java_com_mythcal_mec_MecNative_nativeDevices(JNIEnv* env, jobject, jlong h) {
    Context* c = ctx(h);
    const double now = c->last_ts_s;
    std::string s;
    char line[160];
    for (const mec::DiscoveredDevice& d : c->registry.list(now)) {
        const int age_ms = static_cast<int>((now - d.last_seen_s) * 1000.0);
        std::snprintf(line, sizeof(line), "%llu,%d,%u,%d,%d,%.2f,%.2f,%.2f\n",
                      static_cast<unsigned long long>(d.id), age_ms,
                      static_cast<unsigned>(d.los_state), d.connected ? 1 : 0,
                      d.has_data ? 1 : 0, d.spx, d.spy, d.spz);
        s += line;
    }
    return env->NewStringUTF(s.c_str());
}

JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeConnectDevice(JNIEnv*, jobject, jlong h, jlong id) {
    ctx(h)->registry.connect(static_cast<uint64_t>(id));
}

JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeDisconnectDevice(JNIEnv*, jobject, jlong h, jlong id) {
    ctx(h)->registry.disconnect(static_cast<uint64_t>(id));
}

JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeConnectAll(JNIEnv*, jobject, jlong h) {
    Context* c = ctx(h);
    c->registry.connect_all(c->last_ts_s);
}

JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeDisconnectAll(JNIEnv*, jobject, jlong h) {
    ctx(h)->registry.disconnect_all();
}

// manual=true: only operator-connected devices fuse. false: fuse all visible.
JNIEXPORT void JNICALL
Java_com_mythcal_mec_MecNative_nativeSetManualConnect(JNIEnv*, jobject, jlong h, jboolean manual) {
    ctx(h)->registry.set_manual(manual == JNI_TRUE);
}

JNIEXPORT jint JNICALL
Java_com_mythcal_mec_MecNative_nativeConnectedCount(JNIEnv*, jobject, jlong h) {
    return static_cast<jint>(ctx(h)->registry.connected_count());
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
    np.position = c->node_pos;            // manual co-localization pin (origin by default)
    np.orientation = c->node_orientation; // shared absolute frame (rotation-vector); IMU
                                          // integrator is still used for depth de-rotation
    std::array<mec::WorldKeypoint, mec::kNumKeypoints> kps{};
    for (int i = 0; i < mec::kNumKeypoints; ++i) {
        mec::Keypoint kp = obs.keypoints[mec::kMediapipe33To17[i]];
        kp.id = static_cast<uint8_t>(i);
        kps[i] = c->projector.project(kp, c->intr, np, ts);
    }

    // Multi-observer feed (§15.10): our own observation (when a pose is
    // detected) is fused locally AND broadcast to neighbours; neighbours'
    // beacons are received and fused too — so a phone with no line of sight
    // still renders the pose from others (through-wall).
    c->world.user_neighbours.clear();
    if (kc > 0) {
        const mec::UserStateVector local = mec::sim::pack_beacon(
            kps, c->node_id, ts, static_cast<uint16_t>(c->frame_id),
            mec::LOSState::TRACKING, c->node_pos);
        c->world.user_neighbours.entries.push_back(local);
        c->transport.broadcast(local.payload, static_cast<uint8_t>(mec::LOSState::TRACKING),
                               c->node_pos.x, c->node_pos.y, c->node_pos.z);
    }
#ifdef MEC_USE_MITH
    c->transport.tick(); // drive mith comms: flush sends, drain receives
    // mith auto-announces presence via its beacon; surface its neighbour table
    // into the registry so idle (non-tracking) phones still show up in the scan.
    for (const mec::BeaconObservation& nb : c->transport.neighbours())
        c->registry.observe(nb.sender, ts, nb.spx, nb.spy, nb.spz, nb.los_state, /*has_data=*/false);
#else
    // UDP stopgap: idle phones (no pose) periodically announce presence so peers
    // can discover them, and we reply to received scan probes.
    const uint8_t self_los = static_cast<uint8_t>(kc > 0 ? mec::LOSState::TRACKING
                                                         : mec::LOSState::NO_TARGET);
    if (kc == 0 && ts - c->last_announce_s > 0.5) {
        c->transport.announce_presence(self_los, c->node_pos.x, c->node_pos.y, c->node_pos.z);
        c->last_announce_s = ts;
    }
#endif
    for (const mec::BeaconObservation& rx : c->transport.poll()) {
        // A data beacon carries a valid keypoint frame; presence/probe headers do not.
        uint16_t schema = 0;
        std::memcpy(&schema, rx.payload.data(), sizeof(schema));
        const bool has_data = (rx.kind == mec::kBeaconData) && (schema == mec::kMecSchemaId);

        c->registry.observe(rx.sender, ts, rx.spx, rx.spy, rx.spz, rx.los_state, has_data);
        c->neighbor_seen[rx.sender] = ts;
        if (rx.kind == mec::kBeaconProbe) c->want_announce = true; // reply to scan

        // Manual connect: only fuse devices the operator has connected (§ control).
        if (has_data && c->registry.should_fuse(rx.sender)) {
            mec::UserStateVector usv;
            usv.sender = rx.sender;
            usv.los_state = rx.los_state;
            usv.recv_time_s = ts; // stamp on arrival (no cross-device clock sync)
            usv.spx = rx.spx; usv.spy = rx.spy; usv.spz = rx.spz;
            usv.payload = rx.payload;
            c->world.user_neighbours.entries.push_back(usv);
        }
    }
#ifndef MEC_USE_MITH
    if (c->want_announce) {
        c->transport.announce_presence(self_los, c->node_pos.x, c->node_pos.y, c->node_pos.z);
        c->want_announce = false;
    }
#endif
    // Count distinct neighbour phones heard in the last 2s.
    int nc = 0;
    for (auto it = c->neighbor_seen.begin(); it != c->neighbor_seen.end();) {
        if (ts - it->second > 2.0) it = c->neighbor_seen.erase(it);
        else { ++nc; ++it; }
    }
    c->last_neighbor_count = nc;
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
