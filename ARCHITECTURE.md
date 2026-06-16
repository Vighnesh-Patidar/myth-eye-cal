# Myth-Eye-Cal — Distributed Pose Reconstruction System
## Architecture & Design Document v0.1

> **Repo:** `myth-eye-cal`
> **Namespace:** `mec`
> **Runtime dependency:** `mith-atomas` (coordination + transport layer)
> **Language:** C++17 core, Kotlin/JNI for Android shell, Three.js for browser render
> **Status:** Pre-implementation design document

---

## 0. Name & Philosophy

**Myth-Eye-Cal** is derived from three roots:

- **Mith** — MithAtomas, the coordination substrate underneath
- **Eye** — observation, sensing, line-of-sight awareness
- **Cal** — calculation, calibration, the geometry and fusion math

The name sounds like *mythical* — which is the appropriate connotation for a system that renders a live human pose through a wall on a commodity phone network with no fixed infrastructure.

The core philosophy:

> **Any node can be an observer. Every node sees the whole picture.**

No node is pre-assigned a role. Any phone with line of sight to a target becomes an observer automatically. Any phone without LOS becomes a fusion consumer automatically. Roles are dynamic, self-reported, and can flip within a single session as people and phones move.

---

## 1. Problem Statement

Rendering the position and pose of a person who is not visible to a given device — using only data shared from devices that do have line of sight — across a self-organising network of commodity smartphones.

No fixed infrastructure. No central server. No special hardware. Just phones on a WiFi network running MithAtomas.

What makes this hard:

1. **Pose is not a position.** 17–33 keypoints, each with independent depth, uncertainty, and temporal dynamics. Triangulating a single point is easy. Fusing a full skeletal pose from multiple moving observers with different viewpoints is not.
2. **Observers move.** Unlike fixed sensor arrays, phones are carried by people. The geometry of the observer network changes continuously. The fusion system must handle this without recalibration.
3. **LOS is dynamic.** A phone behind a wall may gain LOS when someone opens a door. A LOS phone may lose it when someone steps in front of it. The system must handle LOS state transitions gracefully.
4. **Latency budget is tight.** Pose rendering feels live at under 100ms end-to-end. Every component in the pipeline competes for that budget.

---

## 2. System Overview

```
┌──────────────────────────────────────────────────────────────────┐
│                        Myth-Eye-Cal                              │
│                                                                  │
│  Any Phone (role determined at runtime)                          │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐     │
│  │                    Observer Pipeline                    │     │
│  │   (active only when LOS detected)                       │     │
│  │                                                         │     │
│  │  Camera2 → FrameMetaBuilder → PoseEstimator → TemporalStereoDepth │  │
│  │               ↓                                         │     │
│  │          KeypointProjector (2D+depth → world-frame 3D)  │     │
│  │               ↓                                         │     │
│  │          KeypointFrame → UserStateVector broadcast       │     │
│  └─────────────────────────────────────────────────────────┘     │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐     │
│  │                    Fusion Pipeline                      │     │
│  │   (active on all nodes always)                          │     │
│  │                                                         │     │
│  │  UserStateVector rx → KeypointAggregator                │     │
│  │               ↓                                         │     │
│  │          MultiObserverFusion (weighted least squares)    │     │
│  │               ↓                                         │     │
│  │          KalmanTracker (per keypoint, 17 independent)   │     │
│  │               ↓                                         │     │
│  │          PoseStateComponent (fused world-frame pose)     │     │
│  └─────────────────────────────────────────────────────────┘     │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐     │
│  │                    Render Pipeline                      │     │
│  │   (WebSocket → browser, or native Android renderer)     │     │
│  │                                                         │     │
│  │  PoseStateComponent → RenderSerializer                  │     │
│  │               ↓                                         │     │
│  │          WebSocket uplink → Three.js skeletal renderer  │     │
│  └─────────────────────────────────────────────────────────┘     │
│                                                                  │
│  ┌─────────────────────────────────────────────────────────┐     │
│  │              MithAtomas Runtime (underneath)            │     │
│  │                                                         │     │
│  │  Entity/Component store, SystemScheduler, NeighbourTable│     │
│  │  UserStateVector channel, clock sync, fault recovery    │     │
│  │  Ed25519 authentication, partition merge                │     │
│  └─────────────────────────────────────────────────────────┘     │
└──────────────────────────────────────────────────────────────────┘
```

Every phone runs all three pipelines. The observer pipeline is gated by LOS detection — if the phone cannot see a target, the pipeline is idle. The fusion pipeline runs on every node always — even LOS nodes fuse all available observations including their own, producing a better estimate than their local view alone. The render pipeline streams the fused pose to whatever display is available.

---

## 3. LOS Detection & Observer Activation

### 3.1 What is LOS?

In Myth-Eye-Cal, LOS means the phone's camera has a person in frame with sufficient confidence from the pose estimator. No RF sensing, no wall detection, no infrastructure. Just: does the camera see a person?

```cpp
namespace mec {

enum class LOSState : uint8_t {
    NO_TARGET   = 0,   // camera sees nothing / confidence below threshold
    ACQUIRING   = 1,   // target detected, depth not yet stable
    TRACKING    = 2,   // target tracked, observations being broadcast
    OCCLUDED    = 3,   // target was tracked, temporarily lost
};

} // namespace mec
```

### 3.2 LOS Confidence Threshold

The pose estimator returns a per-keypoint confidence score (0.0–1.0). A node enters `TRACKING` state when:

- At least 8 of 17 keypoints have confidence > `LOS_CONFIDENCE_THRESHOLD` (default: 0.6)
- Depth estimate has stabilised over at least 5 consecutive frames
- The target has been continuously visible for at least `LOS_ACQUIRE_FRAMES` (default: 10 frames)

Hysteresis: once in `TRACKING`, the node stays in tracking until confidence drops below `LOS_DROP_THRESHOLD` (default: 0.3) for `LOS_DROP_FRAMES` (default: 15 frames) consecutively. This prevents flicker.

### 3.3 LOS State in StateVector

The node's LOS state is carried in the core `StateVector` via `BehaviourStateComponent`. Every other node in the swarm knows which nodes currently have LOS. This is how fusion consumers know which `UserStateVector` entries carry live keypoint data vs stale data.

---

## 4. Observer Pipeline

### 4.1 Camera Capture — Camera2 API

Myth-Eye-Cal captures directly via Android's Camera2 API. No intermediate capture framework. This keeps the dependency footprint minimal and gives direct access to `CaptureResult` metadata needed for the temporal stereo depth pipeline.

```
Camera2 ImageReader (YUV_420_888, 30fps)
        ↓
FrameMetaBuilder (assembles Frame + FrameMeta from CaptureResult + IMU)
        ↓
ObserverPipeline.onFrame(frame, meta)
```

The internal `Frame` and `FrameMeta` types are defined in Myth-Eye-Cal itself:

```cpp
namespace mec {

struct Frame {
    uint8_t*  data;          // YUV_420_888 plane 0 (Y only for LK flow)
    int       width, height;
    int64_t   timestamp_ns;
};

struct FrameMeta {
    float focal_length_px;       // derived from CameraCharacteristics
                                 // focal_length_mm / pixel_size_mm
    float focus_distance_m;      // 1.0f / CaptureResult.LENS_FOCUS_DISTANCE
    float imu_baseline_m;        // from IMUIntegrator since last frame
    float imu_qw, imu_qx,
          imu_qy, imu_qz;        // orientation at capture timestamp
};

} // namespace mec
```

`FrameMetaBuilder` assembles `FrameMeta` by correlating `CaptureResult` timestamps with `IMUIntegrator` output. Both run on a background thread — the main thread never blocks on capture.

GPU path: on devices with OpenCL or Vulkan Compute available, MediaPipe Tasks API automatically uses the GPU for pose inference. Myth-Eye-Cal does not manage this — MediaPipe's runtime handles it transparently via its GPU delegate.

### 4.2 Pose Estimation

Default backend: **MediaPipe Pose Landmarker** (33 keypoints, world landmark mode).
Fallback backend: **MoveNet Thunder** (17 keypoints, faster on weaker hardware).

Both backends are wrapped behind a common interface:

```cpp
namespace mec {

struct Keypoint {
    uint8_t  id;
    float    x, y;          // normalised image coordinates [0,1]
    float    depth_hint;    // monocular depth hint from estimator, metres
    float    confidence;    // per-keypoint visibility confidence [0,1]
};

struct PoseObservation {
    std::array<Keypoint, 33>  keypoints;   // unused slots zeroed
    uint8_t                   keypoint_count;
    uint32_t                  frame_id;
    double                    timestamp_s; // MithAtomas synced clock (§15.2: double, not float)
};

class PoseEstimatorBackend {
public:
    virtual ~PoseEstimatorBackend() = default;
    virtual PoseObservation estimate(const mec::Frame& frame) = 0;
};

} // namespace mec
```

### 4.3 Depth Estimation — Temporal Stereo

Depth is estimated using **temporal stereo** — the phone acts as a moving eye, using IMU dead reckoning to measure the baseline between frames and optical flow to measure disparity. No depth ML model required.

The principle is identical to biological stereo vision: two eye positions separated by a known baseline (interpupillary distance) give depth from disparity. Here the two positions are the same phone at two points in time, baseline measured by the IMU.

```
Frame t=0: capture image I₀, record IMU pose P₀
Phone moves (natural hand/body motion)
Frame t=Δt: capture image I₁, record IMU pose P₁

baseline b  = ||P₁ - P₀||           (IMU dead reckoning, metres)
disparity d = optical flow per keypoint between I₀ and I₁ (pixels)
imu_depth   = (focal_length_px × b) / d
```

**Two independent depth signals are fused per keypoint:**

| Signal | Source | Accuracy | Available when |
|---|---|---|---|
| `lens_prior` | `CaptureResult.LENS_FOCUS_DISTANCE` (dioptres → metres) | ~20cm at 3m | Always (autofocus) |
| `imu_depth` | IMU baseline + Lucas-Kanade optical flow | ~2-5cm at 3m | When phone moves > 5mm |

Fusion via single-measurement Kalman update:

```
depth_fused = kalman_fuse(lens_prior, imu_depth,
                          σ_lens=0.20, σ_imu=0.05)
```

**Fallback rules:**
- `baseline < 5mm` → use `lens_prior` only (phone stationary, no stereo signal)
- `baseline > 500mm` → skip frame (motion too fast, optical flow unreliable)
- `disparity < 0.5px` → use `lens_prior` only (target too far for reliable disparity)

**Why this is better than a depth model:**
- Metric scale from IMU — no scale ambiguity, absolute metres not relative depth
- No TFLite runtime, no bundled model weights
- Accuracy improves with phone motion — the system gets better as observers move
- Works in any lighting condition (depth from geometry, not learned features)

```cpp
namespace mec {

struct IMUFrame {
    float baseline_m;          // scalar distance moved since last camera frame
    float qw, qx, qy, qz;     // orientation quaternion at capture time
    float timestamp_s;
};

class TemporalStereoDepth {
public:
    // Called per frame with current pose observation + IMU data
    // Updates obs.keypoints[i].depth_hint in place
    void resolve(PoseObservation&    obs,
                 const mec::Frame&  prev_frame,
                 const mec::Frame&  curr_frame,
                 const IMUFrame&    imu,
                 float              focal_length_px,
                 float              lens_prior_m);

private:
    // Lucas-Kanade sparse optical flow, 3 pyramid levels, 20 iter max
    // Implemented in C++17 with NEON intrinsics. No OpenCV.
    void compute_disparities(const mec::Frame& prev,
                             const mec::Frame& curr,
                             std::span<const float>   keypoints_prev_px,
                             std::span<float>         disparities_out);

    float kalman_fuse(float prior, float measurement,
                      float sigma_prior, float sigma_meas);

    mec::Frame prev_frame_;
    float             prev_keypoints_px_[33 * 2];  // pre-allocated, no heap
};

} // namespace mec
```

**IMU integration** is handled by `IMUIntegrator` (Android: `SensorManager` at 200Hz, accelerometer + gyro, dead reckoning reset each camera frame). On non-Android targets, IMU data is injected via the `FrameMeta` interface.

### 4.4 Keypoint Projection — Image to World Frame

Each keypoint is projected from image space to world-frame 3D using:

1. Phone's camera intrinsics (focal length, principal point) — read from Android `CameraCharacteristics`
2. Phone's world-frame position from `PositionComponent` (GPS outdoors, or manual pin indoors)
3. Phone's world-frame orientation from `OrientationComponent` (IMU quaternion)
4. Per-keypoint depth from `DepthEstimator`

```
image (u,v) + depth d
        ↓  camera intrinsics
camera-frame ray (cx, cy, cz)
        ↓  camera→body extrinsic (fixed, phone-specific)
body-frame (bx, by, bz)
        ↓  OrientationComponent quaternion
world-frame (wx, wy, wz)
        ↓  + PositionComponent offset
absolute world position
```

Output: `WorldKeypoint` — a 3D point in the shared world frame with a confidence-weighted uncertainty radius.

```cpp
namespace mec {

struct WorldKeypoint {
    uint8_t  id;
    float    wx, wy, wz;      // world-frame position, metres
    float    uncertainty_r;   // 1-sigma uncertainty radius, metres (§15.3: isotropic — limitation)
    float    confidence;
    double   timestamp_s;     // §15.2: double, not float
};

} // namespace mec
```

### 4.5 KeypointFrame Broadcast via UserStateVector

The projected `WorldKeypoint` array is serialised into a `UserStateVector` payload and broadcast via MithAtomas's user beacon channel.

```cpp
namespace mec {

// Fits in UserStateVector's 128-byte payload (verified by static_assert).
//   header        : schema_id(2) + keypoint_count(1) + frame_id(2) +
//                   timestamp_ms(4)                              =  9 bytes
//   keypoints[17] : 17 × (int16 wx,wy,wz [6] + uint8 conf [1])   = 119 bytes
//                                                          total = 128 bytes
//
// REVISION (§15.1): keypoint `id` is now IMPLICIT (array index 0..16) — an
// explicit per-keypoint id pushed the struct to 8 B/kp (136 B + 11 B header =
// 147 B), which overflowed the 128-byte budget and failed the static_assert.
// `frame_id` is uint16 (circular, wraps ~36 min @30fps); the timestamp is a
// uint32 millisecond count, not float (§15.2).
#pragma pack(push, 1)
struct KeypointFramePayload {
    uint16_t schema_id = MEC_SCHEMA_ID;   // 0x4D45 "ME"
    uint8_t  keypoint_count;
    uint16_t frame_id;
    uint32_t timestamp_ms;                // ms since session epoch (49-day range)
    struct PackedKeypoint {
        int16_t wx, wy, wz;    // fixed point, 1cm resolution, range ±327m
        uint8_t confidence;    // 0-255 mapped from [0,1]
    } keypoints[17];
};
#pragma pack(pop)
static_assert(sizeof(KeypointFramePayload) == 128);

} // namespace mec
```

Broadcast rate: matches MediaPipe's inference output rate, target 20–30fps. MithAtomas's user beacon channel carries this separately from the core `StateVector` beacon — no interference with the coordination layer's timing.

---

## 5. Fusion Pipeline

### 5.1 KeypointAggregator

Receives `UserStateVector` entries from `MithAtomas::UserNeighbourTable`. Filters by:

- `schema_id == MEC_SCHEMA_ID` — discard non-Myth-Eye-Cal payloads
- Sender's `LOSState == TRACKING` — discard entries from non-LOS nodes
- `timestamp_s` within fusion window (default: 150ms) — discard stale frames

Produces a per-keypoint list of `WorldKeypoint` observations from all current LOS nodes.

### 5.2 MultiObserverFusion — Weighted Least Squares

For each of the 17 keypoints independently:

Given N observations `{(wx_i, wy_i, wz_i, sigma_i)}` from N LOS nodes:

**Weight per observation:**
```
w_i = confidence_i / (sigma_i² + epsilon)
```

Higher confidence and lower uncertainty → higher weight. Nodes with better viewing angle (keypoint facing the camera vs side-on) naturally have higher confidence from the pose estimator.

**Fused position:**
```
wx_fused = Σ(w_i × wx_i) / Σ(w_i)
wy_fused = Σ(w_i × wy_i) / Σ(w_i)
wz_fused = Σ(w_i × wz_i) / Σ(w_i)
```

Fused uncertainty:
```
sigma_fused = 1 / sqrt(Σ(1 / sigma_i²))
```

Accuracy improves as `1/sqrt(N)` with number of independent observers. Two observers: ~30% uncertainty reduction over single best observer. Four observers: ~50% reduction.

**Geometry bonus:** Observers at orthogonal viewing angles contribute independent depth information. The weighting scheme naturally captures this — when two observers are at 90° to each other, each has high confidence on the axes they see clearly and low confidence on depth, and the fusion combines their independent axes.

### 5.3 KalmanTracker — Per Keypoint

17 independent Kalman filters, one per keypoint. State vector per filter:

```
x = [wx, wy, wz, vx, vy, vz]ᵀ   (position + velocity, world frame)
```

Process model: constant velocity with Gaussian process noise tuned for human movement (max acceleration ~5 m/s²).

Update: called with fused `WorldKeypoint` from MultiObserverFusion at each fusion tick.

Predict: called at render rate (60fps target) between fusion updates. This is what keeps the render smooth even when observations are at 20fps — the Kalman filter predicts forward and the renderer consumes predictions.

```cpp
namespace mec {

class KeypointKalmanTracker {
public:
    void update(const WorldKeypoint& fused_obs);
    WorldKeypoint predict(float timestamp_s) const;
    bool is_initialised() const;
    float confidence() const;   // decays when updates stop arriving
};

} // namespace mec
```

### 5.4 PoseStateComponent

The output of the fusion pipeline. Written by `PoseFusionSystem`, read by `RenderPipeline`.

```cpp
namespace mec {

struct PoseStateComponent : mith::ColdComponent<PoseStateComponent> {
    std::array<WorldKeypoint, 17>  keypoints;
    float                          last_update_s;
    uint8_t                        observer_count;   // how many LOS nodes contributed
    float                          mean_confidence;
    bool                           is_valid;
};

} // namespace mec
```

---

## 6. Render Pipeline

### 6.1 Design

Two render targets, both consuming `PoseStateComponent`:

**Primary: Browser via WebSocket**
- MithAtomas WebSocket transport (V2.0 roadmap item, needed here)
- Browser runs Three.js skeletal renderer
- Any device on the same network opens a URL and sees the live pose
- No app install required

**Secondary: Native Android**
- OpenGL ES skeletal renderer embedded in the Android app
- Lower latency than browser path (~10ms vs ~30ms)
- Better for the phone that needs to act on the pose data in real time

### 6.2 Skeletal Model

17-keypoint skeleton (MediaPipe convention, falls back gracefully from 33):

```
Nose
├── Left Eye, Right Eye
├── Left Ear, Right Ear
Left Shoulder ── Right Shoulder
├── Left Elbow      ├── Right Elbow
│   └── Left Wrist  │   └── Right Wrist
Left Hip ── Right Hip
├── Left Knee       ├── Right Knee
│   └── Left Ankle  │   └── Right Ankle
```

Rendered as: spheres at joints, cylinders for bones, colour-coded by confidence (green = high, yellow = medium, red = low / predicted).

### 6.3 WebSocket Pose Serialisation

```json
{
  "type": "pose_frame",
  "timestamp_s": 1234.567,
  "observer_count": 3,
  "keypoints": [
    {"id": 0, "wx": 1.23, "wy": 0.45, "wz": 2.10, "conf": 0.92},
    ...
  ]
}
```

Sent at Kalman prediction rate (target 60fps). ~800 bytes per frame. At 60fps: ~48KB/s per connected browser. Trivial on WiFi.

### 6.4 Three.js Renderer

Single HTML file, no build step. Opens in any browser.

```
myth-eye-cal-viewer.html
├── Three.js (CDN)
├── WebSocket client → ws://[phone-ip]:8080/pose
├── SkeletonRenderer
│   ├── 17 sphere meshes (joints)
│   ├── 16 cylinder meshes (bones)
│   └── confidence colour mapping
├── CameraControls (orbit, zoom)
└── Stats overlay (observer count, latency, confidence)
```

---

## 7. Latency Budget

Target: under 100ms from capture to render update.

| Stage | Budget | Notes |
|---|---|---|
| Capture → frame ready | 33ms | 30fps camera |
| Camera2 → FrameMetaBuilder | 5ms | YUV extraction, IMU correlation |
| Pose estimation | 15ms | MediaPipe on mid-range Android |
| Temporal stereo depth | 5ms | LK optical flow (NEON) + IMU fuse + lens prior |
| Keypoint projection | 1ms | Pure math, camera intrinsics |
| UserStateVector broadcast | 5ms | MithAtomas user beacon |
| Network transit | 5ms | Local WiFi |
| KeypointAggregator + fusion | 3ms | Weighted least squares, 17 keypoints |
| Kalman update | 1ms | 17 independent filters |
| Render serialisation + WebSocket | 5ms | JSON encode + send |
| Browser render | 16ms | 60fps frame |
| **Total** | **94ms** | Tight but achievable |

Temporal stereo depth runs in the same 5ms slot previously occupied by the depth model — Lucas-Kanade on 17 sparse keypoints with NEON intrinsics is faster than a TFLite forward pass and produces metric depth directly.

The Kalman predictor decouples fusion rate from render rate. Fusion runs at 20–30fps (inference rate). Renderer runs at 60fps consuming Kalman predictions between fusion updates. Perceived latency = capture-to-render, not fusion rate.

---

## 8. Component Model (MithAtomas integration)

Myth-Eye-Cal registers the following components on each entity (phone node):

**Hot components (added to MithAtomas hot component set):**

| Component | Fields | Owner |
|---|---|---|
| `LOSStateComponent` | `LOSState state, float confidence, uint32_t frame_id` | ObserverPipeline |

**Cold components:**

| Component | Fields | Owner |
|---|---|---|
| `PoseStateComponent` | 17 `WorldKeypoint` + metadata | PoseFusionSystem |
| `CameraIntrinsicsComponent` | focal length, principal point, distortion | Init |
| `ObserverMetricsComponent` | fps, mean inference latency, drop rate | ObserverPipeline |

---

## 9. Systems

All systems registered with MithAtomas `SystemScheduler`. Dependency graph ensures correct execution order.

| System | Reads | Writes | Rate |
|---|---|---|---|
| `ObserverActivationSystem` | LOSStateComponent, NeighbourTable | LOSStateComponent | 10Hz |
| `KeypointBroadcastSystem` | LOSStateComponent, CameraIntrinsics, Orientation, Position | UserStateVector (outbound) | 20–30Hz |
| `KeypointAggregatorSystem` | UserNeighbourTable | internal buffer | 20–30Hz |
| `PoseFusionSystem` | internal buffer | PoseStateComponent | 20–30Hz |
| `KalmanPredictSystem` | PoseStateComponent | PoseStateComponent | 60Hz |
| `RenderSerialiserSystem` | PoseStateComponent | WebSocket outbound | 60Hz |

`KalmanPredictSystem` and `RenderSerialiserSystem` run at 60Hz independently of the inference-rate systems. MithAtomas's async DAG scheduler handles the mixed-rate execution natively.

---

## 10. Repository Structure

```
myth-eye-cal/
├── CMakeLists.txt
├── README.md
├── ARCHITECTURE.md               ← this document
├── LICENSE                       ← Apache 2.0
│
├── include/
│   └── mec/
│       ├── mec.h                 ← single include
│       ├── observer/
│       │   ├── los_detector.h
│       │   ├── pose_estimator.h
│       │   ├── depth_estimator.h
│       │   └── keypoint_projector.h
│       ├── fusion/
│       │   ├── keypoint_aggregator.h
│       │   ├── multi_observer_fusion.h
│       │   └── keypoint_kalman.h
│       ├── components/
│       │   ├── los_state_component.h
│       │   ├── pose_state_component.h
│       │   └── camera_intrinsics_component.h
│       ├── systems/
│       │   ├── observer_activation_system.h
│       │   ├── keypoint_broadcast_system.h
│       │   ├── keypoint_aggregator_system.h
│       │   ├── pose_fusion_system.h
│       │   ├── kalman_predict_system.h
│       │   └── render_serialiser_system.h
│       └── render/
│           ├── websocket_render_server.h
│           └── pose_serialiser.h
│
├── src/
│   ├── observer/
│   ├── fusion/
│   ├── systems/
│   └── render/
│
├── android/
│   ├── app/                      ← Android Studio project
│   │   ├── jni/                  ← JNI bridge to C++ core
│   │   └── kotlin/               ← Android shell, camera access
│   └── CMakeLists.txt
│
├── viewer/
│   └── myth-eye-cal-viewer.html  ← Three.js browser renderer
│
├── examples/
│   └── sim_pose_demo/            ← simulated LOS node, no camera needed
│
└── tests/
    ├── unit/
    │   ├── test_keypoint_projector.cpp
    │   ├── test_multi_observer_fusion.cpp
    │   └── test_kalman_tracker.cpp
    └── integration/
        └── test_fusion_pipeline.cpp
```

---

## 11. Dependencies

| Dependency | Use | Bundled |
|---|---|---|
| `mith-atomas` | coordination, transport, clock sync | submodule |
| MediaPipe (Android AAR) | pose estimation | Android only |
| Android SensorManager | IMU data at 200Hz (accelerometer + gyro) | Android only, no library |
| Three.js | browser skeletal renderer | CDN |
| uWebSockets | WebSocket server (C++) | vendored |
| nlohmann/json | pose frame serialisation | vendored header |

No ROS. No OpenCV. No TFLite. No depth model weights. No external capture framework. Depth is pure geometry — Lucas-Kanade optical flow (hand-rolled C++17 NEON) + IMU dead reckoning + lens focus prior. GPU inference is handled transparently by MediaPipe's Tasks API GPU delegate.

---

## 12. Roadmap

### v0.1 — Fusion Core (no camera)
- [ ] `KeypointAggregatorSystem` + `PoseFusionSystem` + `KalmanTracker`
- [ ] Simulated LOS node (generates synthetic keypoint observations)
- [ ] `myth-eye-cal-viewer.html` Three.js renderer
- [ ] WebSocket render server
- [ ] Unit tests: fusion math, Kalman tracker, projector
- [ ] Integration test: 3 simulated LOS nodes → fused pose → browser render

### v0.2 — Observer Pipeline
- [ ] MediaPipe Pose integration (Android, Tasks API)
- [ ] `TemporalStereoDepth` — Lucas-Kanade optical flow (C++17 NEON, no OpenCV)
- [ ] `IMUIntegrator` — Android SensorManager 200Hz, dead reckoning per frame
- [ ] Kalman fusion of lens prior + IMU depth per keypoint
- [ ] `KeypointProjector` with camera intrinsics from `CameraCharacteristics`
- [ ] `LOSDetector` with hysteresis (§3.2 thresholds)
- [ ] Android JNI bridge (fusion core + MithAtomas as .so via NDK)

### v0.3 — Full System
- [ ] End-to-end: phone camera → fused pose → browser render
- [ ] Multi-phone test (minimum 2 LOS + 1 non-LOS)
- [ ] Latency measurement and logging
- [ ] LOS state transition testing (phone moves behind wall mid-session)

### v1.0 — Stable
- [ ] API stability
- [ ] Doxygen
- [ ] CI: Linux x86 (fusion core) + Android aarch64 (observer pipeline)
- [ ] Accuracy benchmark: fused pose error vs ground truth at 2/3/4 observers
- [ ] Single-observer fallback mode (degrades gracefully with 1 LOS node)

---

## 13. Design Constraints & Non-Goals

**Constraints:**
- Fusion core (C++) has zero Android dependencies — testable on Linux
- No heap allocation in `PoseFusionSystem` or `TemporalStereoDepth` hot path — all buffers pre-allocated
- `KeypointFramePayload` must fit in `UserStateVector` 128-byte payload — enforced by `static_assert`
- No OpenCV anywhere — Lucas-Kanade is hand-rolled C++17 with NEON intrinsics
- No TFLite, no depth model weights — depth is pure geometry + IMU + optics
- Latency target: 100ms capture-to-render on mid-range Android (Snapdragon 778G class)
- `TemporalStereoDepth::resolve()` must complete in < 5ms on target hardware
- IMU integration resets every camera frame — no long-term drift accumulation

**Non-goals for v1.0:**
- Multi-person tracking (single target assumed; multi-target is post-v1.0)
- RF/WiFi CSI through-wall sensing (out of scope — LOS detection is camera-only)
- Outdoor GPS-accuracy positioning (indoor positioning via manual pin or WiFi RSSI; GPS optional)
- Video streaming (Myth-Eye-Cal transmits keypoints, not frames — video is out of scope for v1.0)
- Stereo camera support (temporal stereo on a single camera is the design; true stereo is post-v1.0)
- Depth estimation when phone is stationary (lens prior only in that case — by design)

---

## 14. The Stack in Full

```
myth-eye-cal          ← you are here: distributed pose fusion + render
      │                    Camera2 capture, temporal stereo depth,
      │                    MediaPipe pose, OpenGL ES render
      │
mith-atomas           ← coordination, transport, clock sync, auth, fault recovery
      │
      └── any phone, any role, any wall
```

Three repos. Three layers. One system that renders a live human pose through a wall on commodity smartphones with no fixed infrastructure and no central server.

---

## 15. Design Review & Revisions

Findings from the v0.1 implementation pass. Each was reflected back into the
structs above; the rationale is recorded here.

### 15.1 KeypointFramePayload overflowed its 128-byte budget *(critical, fixed)*

As originally written the struct was **147 bytes**, not 128: the header was 11
bytes (`uint32 frame_id` + `float timestamp`) and each `PackedKeypoint` was 8
bytes (explicit `uint8 id` + 3×`int16` + `uint8`), so `17 × 8 + 11 = 147`. The
`static_assert(... <= 128)` would have failed to compile.

**Fix:** drop the explicit per-keypoint `id` (it is the array index 0..16),
shrink `frame_id` to `uint16`, and carry the timestamp as a `uint32`
millisecond count. New layout is exactly **128 bytes** (9 B header + 17×7 B),
and the assert is tightened to `== 128`. `frame_id` is treated as circular
(wraps ~36 min at 30 fps); receivers compare modulo-2¹⁶.

### 15.2 `float` timestamps cannot hold a synced clock *(critical, fixed)*

`PoseObservation`, `WorldKeypoint`, and the wire payload used `float
timestamp_s`. `float32` has ~7 significant digits, so an absolute synced-clock
value (~1.7×10⁹ s) is only representable to ~128 s resolution — and even a
session-relative value degrades to ~ms resolution after a few hours. With a
150 ms fusion window (§5.1) and a 100 ms latency budget (§7), this silently
corrupts windowing and ordering.

**Fix:** in-memory timestamps are now `double` seconds. The wire payload uses
`uint32` milliseconds since a session epoch (1 ms resolution, 49-day range) to
stay within the 4-byte field.

### 15.3 Scalar `uncertainty_r` cannot model anisotropic depth error *(upgrade, deferred)*

§5.2's "geometry bonus" states that observers at orthogonal angles contribute
independent depth information. A single **isotropic scalar** `uncertainty_r`
per keypoint cannot express that — monocular / temporal-stereo error is large
*along the camera ray* and small *laterally*, an anisotropic covariance the
scalar collapses. The v0.1 fuser is therefore a correct scalar inverse-variance
weighted mean, but it does **not** realise the per-axis geometry bonus as
described.

**Planned upgrade (post-v0.1):** carry a per-observation 3×3 covariance (or at
minimum an along-ray vs lateral σ split) and fuse with the full information
form `Σ Σᵢ⁻¹` / `Σ Σᵢ⁻¹ xᵢ`. The `WorldKeypoint` API and `MultiObserverFusion`
are documented as the extension points. Tracked as a v1.0 accuracy item.

### 15.4 33→17 keypoint mapping was unspecified *(minor, fixed)*

The doc defaults the estimator to MediaPipe's 33 landmarks (§4.2) but fuses and
broadcasts 17 (§4.5, §6.2) without stating the mapping. The implementation adds
an explicit `kMediapipe33To17` index table (`keypoint_projector.h`). Note one
consequence of the strict 17-slot skeleton: only one ankle survives the cut
(slot 16); a future revision may prefer 18 slots or re-pack the face points.

---

*Document version: 0.3.1 — Design review (§15): 128-byte payload fix, double
timestamps, anisotropic-fusion upgrade noted, 33→17 mapping specified*
*Document version: 0.3.0 — Thundercam dependency removed; Camera2 direct capture; self-contained*
*Authors: Vighnesh Patidar*
*Depends on: mith-atomas v1.0+*
*Repository: github.com/Vighnesh-Patidar/myth-eye-cal*
