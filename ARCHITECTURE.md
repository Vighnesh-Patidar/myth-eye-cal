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

> **Measured (2026-06-17).** The budget above is a *target*. The fusion-core
> stages are now measured (host i5-11260H): scalar fuse 19 ns/keypoint,
> anisotropic fuse 64 ns/keypoint, Kalman update 102 ns, Kalman predict 28 ns,
> full 17-keypoint 3-observer frame **5.1 µs** end to end — ~4 orders of
> magnitude under the 3 ms fusion budget row. On device, fused poses stream to
> the host at **11.4 Hz** (mean 88.8 ms between frames) driven by camera +
> MediaPipe `lite`, consistent with the <100 ms capture-to-render target. Full
> numbers + method: [`docs/METRICS_REPORT.md`](docs/METRICS_REPORT.md).

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
| ~~uWebSockets~~ → hand-rolled | WebSocket server (C++) | none (§15.5) |
| nlohmann/json | pose frame serialisation | vendored header |

No ROS. No OpenCV. No TFLite. No depth model weights. No external capture framework. Depth is pure geometry — Lucas-Kanade optical flow (hand-rolled C++17 NEON) + IMU dead reckoning + lens focus prior. GPU inference is handled transparently by MediaPipe's Tasks API GPU delegate.

---

## 12. Roadmap

### v0.1 — Fusion Core (no camera)
- [x] `MultiObserverFusion` + `KeypointKalmanTracker` + all six §9 `System`
      wrappers, wired through the first-party `mec` fusion runtime (§15.6)
- [x] Simulated LOS node (`sim/synthetic_pose.h`, `sim_pose_demo`)
- [x] `myth-eye-cal-viewer.html` Three.js renderer
- [x] WebSocket render server (hand-rolled, §15.5; `render_server_demo`)
- [x] Unit tests: fusion math, Kalman tracker, projector
- [x] Integration test: 3 simulated LOS nodes → fused pose (2.5cm mean error),
      live WebSocket → browser render path verified

### v0.2 — Observer Pipeline
- [ ] MediaPipe Pose integration (Android, Tasks API) — needs Android
- [x] `TemporalStereoDepth` — Lucas-Kanade optical flow (portable C++17
      reference, no OpenCV; NEON deferred to Android build, §15.7)
- [x] `IMUIntegrator` — strapdown dead-reckoning math done + tested on Linux
      (§15.8); only the SensorManager 200Hz sampling source needs Android
- [x] Kalman fusion of lens prior + IMU depth per keypoint
- [x] `KeypointProjector` with camera intrinsics (done in v0.1)
- [x] `LOSDetector` with hysteresis (§3.2 thresholds)
- [ ] Android JNI bridge (fusion core + MithAtomas as .so via NDK) — needs Android

### v0.3 — Full System
- [~] Android shell scaffolded; **debug APK builds** (NDK `libmec_jni.so`
      bundling the full tested core + JNI bridge; Kotlin Camera2 + SensorManager
      + MediaPipe) — see `android/` and §15.9. On-device run pending deploy.
- [ ] End-to-end on-device: phone camera → fused pose → browser render
- [ ] Multi-phone test (minimum 2 LOS + 1 non-LOS) — needs real mith-atomas
- [ ] Latency measurement and logging
- [ ] LOS state transition testing (phone moves behind wall mid-session)

### v1.0 — Stable
- [ ] API stability
- [ ] Doxygen (deferred)
- [x] CI: GitHub Actions — C++ core build+test (Linux) + Android debug APK
      (`.github/workflows/ci.yml`)
- [~] Accuracy benchmark: methodology defined (`docs/ACCURACY_METHODOLOGY.md`);
      run pending 4-device hardware
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

### 15.3 Anisotropic covariance fusion *(done)*

§5.2's "geometry bonus" requires that observers at orthogonal angles contribute
independent depth information. A single **isotropic scalar** `uncertainty_r`
could not express that — monocular / temporal-stereo error is large *along the
camera ray* and small *laterally* — so the original scalar weighted mean did
**not** realise the per-axis bonus.

**Done.** `WorldKeypoint` now carries a view ray (`rx,ry,rz`) and an along-ray
`depth_uncertainty` (with `uncertainty_r` as the lateral σ).
`MultiObserverFusion::fuse_anisotropic()` fuses in the **information form**
`x = (Σ Λᵢ)⁻¹ (Σ Λᵢ xᵢ)`, with `Λᵢ = conf·[ (1/σ_lat²)(I − r rᵀ) + (1/σ_depth²) r rᵀ ]`
(isotropic observations, zero ray, reduce to `conf/σ² · I`). In the ECS path the
`KeypointAggregatorSystem` reconstructs each view ray from the sender's world
position (carried on the `UserStateVector`, since the 128-byte payload has no
room — §15.6) and `PoseFusionSystem` calls `fuse_anisotropic`.

Verified: the orthogonal-observer unit test fuses to <2 cm where the naive mean
sits >15 cm off (>5× better, `test_anisotropic_fusion`); end-to-end, three
diverse-angle observers with 12 cm per-view depth noise fuse to ~2.0 cm mean
error (`test_ecs_pipeline`).

**Resolved (was a residual):** the per-keypoint tracker is now a coupled
**6-state (position + velocity) Kalman filter that accepts the full fused 3×3
covariance** as its measurement noise `R`, so the anisotropy survives temporal
filtering — the gain is direction-dependent (sharp lateral axes are trusted, the
ambiguous depth axis is smoothed harder) and cross-axis correlations persist.
`fuse_anisotropic()` now emits the fused covariance `(ΣΛ_i)⁻¹` on
`WorldKeypoint::cov_*`; `KeypointKalmanTracker` consumes it and re-emits the
predicted position covariance from `predict()`. When an observation carries no
covariance (`cov_xx ≤ 0`) `R` falls back to `uncertainty_r² · I`, reducing
*exactly* to the previous three decoupled per-axis filters (back-compatible).
Verified by `test_kalman_tracker` (anisotropy preserved at steady state;
direction-dependent gain; zero cross-covariance on the isotropic path).

### 15.5 Render server: uWebSockets replaced by a hand-rolled RFC 6455 server *(deviation)*

§11 listed vendored **uWebSockets** (which pulls uSockets + zlib) for the
render path. For v0.1 the render server only does a handshake and broadcasts
small JSON text frames to a handful of LAN browser clients — server→client
only, no TLS, no per-message-deflate. A ~300-line dependency-free `poll()`
server (`websocket_render_server.{h,cpp}`) covers that and keeps the project
consistent with §11's "no heavy deps" / "hand-rolled" ethos and testable
offline (loopback integration test). The public interface
(`start`/`poll_events`/`broadcast`/`stop`) is deliberately small so uWebSockets
can be swapped back in if TLS or high client counts are later required.
Likewise `nlohmann/json` is avoided — the §6.3 frame is emitted by the
header-only `pose_serialiser.h`.

### 15.4 33→17 keypoint mapping was unspecified *(minor, fixed)*

The doc defaults the estimator to MediaPipe's 33 landmarks (§4.2) but fuses and
broadcasts 17 (§4.5, §6.2) without stating the mapping. The implementation adds
an explicit `kMediapipe33To17` index table (`keypoint_projector.h`). Note one
consequence of the strict 17-slot skeleton: only one ankle survives the cut
(slot 16); a future revision may prefer 18 slots or re-pack the face points.

### 15.6 ECS systems on the first-party `mec` fusion runtime *(resolved)*

The §9 systems run on a small, first-party Entity/Component/System runtime
(`include/mec/ecs/world.h`, namespace `mec`): `World` (entity/component store),
`SystemScheduler` (mixed-rate, dependency-ordered), `NeighbourTable`,
`UserNeighbourTable`, the user beacon channel, and the core
`Position`/`Orientation`/`BehaviourState` components. This hosts all six systems
plus the `LOSDetector` (§3.2), built and tested on Linux. Originally a "mock
mith" stand-in, it is now the app's permanent fusion runtime: mith-atomas is
N=1-per-World and cannot host the many-entity fusion graph (§15.13), so the two
layers coexist — `mec::ecs` for fusion, `mith::` for coordination/transport. The
old `mock/` directory has been removed; `mec_core` owns the ECS header.

Two findings surfaced while wiring it:

- **Node-local scratch components.** The aggregator→fusion buffer, the Kalman
  filter bank, and the observer-pipeline observation seam are node-local and
  must NOT be replicated by the coordination layer — so they are deliberately
  absent from §8's StateVector table (`internal_components.h`).
- **The wire payload carries no per-observation uncertainty.** The 128-byte
  budget (§4.5) leaves room only for confidence, not `uncertainty_r`. The
  aggregator therefore reconstructs a nominal σ from confidence
  (`σ ≈ 0.03 / confidence`) before fusion. This is lossy; if accuracy demands
  it, a future payload could spend one byte on a quantised σ (dropping to 16
  keypoints, or shrinking the frame_id) — tracked alongside §15.3.

### 15.7 Temporal-stereo depth: portable LK + a physical caveat *(deviation + upgrade)*

`TemporalStereoDepth` + `LucasKanade` implement §4.3 (metric depth from IMU
baseline × optical-flow disparity, Kalman-fused with the lens prior). Two notes:

- **Portable scalar LK, not NEON.** The reference Lucas-Kanade
  (`lucas_kanade.{h,cpp}`) is portable C++17 so it builds and is tested on Linux
  x86 (recovers known sub-pixel shifts; depth tests recover `f·b/disparity`).
  §4.3/§13's NEON intrinsics are an ARM build-time optimisation of the inner
  window loop that does not change the interface; deferred to the Android build.
  `Frame::data` is `const` here (read-only) — a harmless tightening of §4.1.

- **Disparity is contaminated on a moving subject (physical limitation).**
  Temporal stereo recovers depth from camera-*translation* parallax of *static*
  points. Two effects break that assumption for this application:
  *(a)* the **subject is moving** — a keypoint's optical flow is camera parallax
  *plus* the subject's own image motion, so `depth = f·b/disparity` is biased;
  *(b)* **inter-frame camera rotation** produces flow unrelated to depth and must
  be de-rotated first.

  **Done (b) — inter-frame de-rotation.** `IMUFrame` now carries the inter-frame
  rotation delta (`dqw..dqz`, body frame) computed by `IMUIntegrator::consume()`;
  `resolve()` takes full `CameraIntrinsics` and, per keypoint, predicts the
  rotation-only displacement via the infinite homography `K·R·K⁻¹`, subtracts it
  from the measured LK flow, and uses only the translational residual for depth.
  `Config::cam_to_body` brings the body-frame rotation into the camera frame.
  Verified: a pure camera rotation now yields sub-threshold residual (→ lens
  prior) instead of a bogus shallow depth, and rotation+translation recovers the
  correct depth (`test_temporal_stereo`).

  **Done (a) — subject-motion gating (epipolar).** `IMUFrame` now also carries
  the unit translation direction (`td*`, body frame, from `IMUIntegrator`).
  After de-rotation, a static point's flow must lie along the camera-translation
  motion field `d = (-fx·Tx + (u-cx)·Tz, -fy·Ty + (v-cy)·Tz)`. `resolve()`
  projects the de-rotated flow onto that epipolar direction: the parallel
  component gives depth (`Z = baseline·|d| / parallax`, which also generalises
  the old `f·b/disp` to non-lateral / focus-of-expansion motion), and a
  perpendicular component above `Config::motion_gate_px` flags subject motion →
  lens-prior fallback. Verified in `test_temporal_stereo` (off-epipolar flow
  gated; on-epipolar flow recovers depth).

  **Residual limitation.** Epipolar gating only rejects the subject-motion
  component *perpendicular* to the epipolar line; subject motion *along* it is
  geometrically indistinguishable from depth parallax and still biases depth.
  Fully removing it needs cross-observer reconciliation (compare against the
  multi-observer fused pose) — tracked as a v1.0 item alongside §15.3. The
  gating also trusts the IMU translation direction; bad direction ⇒ bad gating.

### 15.8 IMU integrator: "reset each frame" clarified + drift caveat *(clarification + upgrade)*

`IMUIntegrator` (`imu_integrator.{h,cpp}`) is a strapdown dead-reckoning
integrator: exponential-map gyro integration for orientation, gravity-
compensated accel (`a = R·f + g_world`) double-integrated for displacement.
Tested on Linux with synthetic samples (90° yaw, gravity rejection, constant-
velocity baseline). Two notes:

- **"Reset each camera frame" (§4.3/§13) means reset *displacement*, not
  *velocity*.** Zeroing velocity each frame would report a zero baseline for a
  phone in steady motion (constant velocity ⇒ zero accel ⇒ no displacement
  rebuilds) — exactly the case temporal stereo needs. So `consume()` resets the
  displacement accumulator (bounding position drift across frames, the §13
  intent) but carries velocity and orientation as continuous physical state.
- **Velocity drift bounded by ZUPT (done).** Resetting displacement bounds
  *position* drift, but accelerometer bias still integrates into *velocity*.
  `IMUIntegrator` now applies a **zero-velocity update**: when specific force
  ≈ g and gyro ≈ 0 for `still_samples` consecutive samples it declares the
  device at rest and zeroes velocity (and ignores the biased specific force),
  so bias cannot drift velocity across rest periods. Verified
  (`test_imu_integrator` cases 5–6): a biased accelerometer at rest drifts
  ~0.05 m without ZUPT but ~0 with it, and ZUPT does not fire under genuine
  acceleration.
  **Fundamental limitation:** an accelerometer cannot distinguish *rest* from
  *constant velocity* (both give |f| = g, ω = 0), so ZUPT also fires during
  rare perfectly-constant-velocity translation and would zero a real velocity.
  Benign for this app (deliberate translation carries jerk; rest correctly
  wants v = 0; the constant-velocity baseline is small and degrades gracefully
  to the lens prior), but it is why steady-coast integration must disable ZUPT.
  A visual-velocity cross-check (optical-flow scale → velocity) would remove the
  ambiguity — tracked as a v1.0 item. The SensorManager 200 Hz sampling source
  is the only Android-specific piece still deferred; the math here is complete.

### 15.9 Android port — scaffolded and building *(milestone)*

The `android/` module wires the device shell to the tested core: a JNI bridge
(`cpp/mec_jni.cpp`) whose NDK `CMakeLists.txt` compiles the seven portable
`mec_core` sources + the bridge into `libmec_jni.so` (arm64-v8a), and a Kotlin
shell — `CameraController` (Camera2 YUV_420_888), `ImuController`
(SensorManager → `IMUIntegrator`), `PoseEstimator` (MediaPipe Pose Landmarker),
`MainActivity` (orchestration + device IP for the browser). The **debug APK
builds** (verified on Linux with a home-dir JDK17/SDK/NDK toolchain): the native
`.so` and all Kotlin compile clean; the MediaPipe model is bundled.

On-device data flow (single device): Camera2 → MediaPipe (Kotlin) →
`nativeOnFrame` → temporal-stereo depth + projection + fusion + Kalman →
WebSocket render to the browser; SensorManager → `nativeOnImuSample` at ~200Hz.

Bring-up shortcuts to revisit (tracked in `android/README.md`): pose runs on a
grayscale bitmap; `lensPriorM` is constant (wire `LENS_FOCUS_DISTANCE`); the
node is pinned at the origin. Multi-device fusion rides the real mith-atomas
transport (`MEC_USE_MITH`, on for Android, §15.13); the local fusion ECS is
mec's own runtime (`include/mec/ecs`). On-device run is pending a deploy
(VirtualBox USB passthrough or copying the APK to the host).

### 15.10 UDP multi-device transport (stopgap for mith-atomas) *(feature)*

`UdpBeaconTransport` (`transport/udp_beacon_transport.{h,cpp}`) carries the user
beacon channel (§4.5) across phones until the real mith-atomas transport is
vendored. Each node broadcasts its 128-byte `KeypointFramePayload` plus sender
id, LOS state, and world position over Wi-Fi UDP (port 8079); every node fuses
its own observation **and** all neighbours' — so a phone with no line of sight
renders the pose from others (through-wall). No clock sync assumed: the receiver
stamps arrival time (the fusion window uses that). On Android a Wi-Fi
`MulticastLock` is acquired so the driver delivers broadcast packets. Verified on
Linux (`test_udp_transport`, two instances exchange a beacon) and starts
on-device (`UDP beacon transport on udp/8079`). The interface mirrors the beacon
channel so mith-atomas swaps in cleanly.

**Open — shared world frame (co-localization).** Multi-device fusion is only
coherent if all phones agree on a common frame. Today each phone projects in its
own camera frame pinned at the origin (`nativeSetNodePose` allows a manual
position pin, but cross-device *orientation* alignment is unsolved without a
shared reference — IMU yaw drifts, no magnetometer fusion). Without
co-localization, fused multi-phone poses will not align. Real fix: a shared
anchor or mutual-observation calibration. Tracked for v0.3.

### 15.11 Single-device pose quality — deferred improvements *(future work)*

A single phone is **not** the target (the design is multi-view geometric); its
pose is intentionally left limited for now. Known limitations + planned fixes:

- **Degenerate depth → flat pose.** `lensPriorM` is a constant and temporal
  stereo needs phone translation + a static subject, so keypoints collapse to
  ~the lens-prior distance. Fix: feed `CaptureResult.LENS_FOCUS_DISTANCE`
  per-frame; optionally an opt-in single-device mode using MediaPipe's metric
  world-landmarks for a real 3D skeleton (deviates from the §4.3/§13 "no depth
  model" ethos — hence opt-in).
- **Orientation drift → skeleton wanders.** The world projection uses raw
  dead-reckoned gyro (no gravity/magnetometer correction). Fix: a
  gravity-referenced orientation (accelerometer tilt — drift-free pitch/roll;
  yaw needs a magnetometer) for projection, keeping the IMU *delta* for depth
  de-rotation only.
- **Grayscale bitmap** for MediaPipe lowers landmark accuracy → full YUV→RGB.

These are tracked for v0.3+; the multi-phone path (§15.10) is where accuracy
comes from.

### 15.12 Co-localization — shared frame via rotation-vector orientation *(feature)*

Multi-phone fusion needs a common world frame (§15.10). Solved without extra
hardware: every phone takes its **absolute orientation from the rotation-vector
sensor** (fused accelerometer + gyroscope + magnetometer → referenced to gravity
+ magnetic north), fed to the projector as the node orientation
(`nativeSetOrientation`, `OrientationController`). Because the reference is
identical on every phone and the same APK shares the camera→body convention, all
phones' world frames are **aligned by construction** — and the orientation is
**drift-free** (this also retires the §15.11 drift item). The IMU integrator is
still used internally for temporal-stereo de-rotation. Per-phone **position** is
a manual pin entered on screen (x/y/z metres from a chosen origin →
`nativeSetNodePose`), or an optional **GPS fill** (`LocationController`): every
phone taps "GPS origin" at one shared spot, then "GPS fill" at its position,
giving ENU metres from the common origin (matching the rotation-vector ENU
orientation). GPS is coarse (~3-10 m) and poor indoors, so manual pins remain
better for room-scale demos; GPS suits outdoor / large layouts.

### 15.13 Real mith-atomas comms integration *(feature, behind a flag)*

`MithRuntime` (`transport/mith_runtime.{h,cpp}`) backs the user beacon channel
with the real `mith-atomas` runtime instead of the UDP stopgap (§15.10). It uses
**both** mith channels as designed (docs/MITH_INTEGRATION.md): the auto
`BeaconSystem` replicates this node's `Position` + `BehaviourState` (LOS), our
128-byte keypoint frame rides a `CUSTOM` `Message`, and `poll()` drains received
payloads and pairs each with its sender's neighbour-table entry (position + LOS)
— emitting the same `BeaconObservation` the fusion path already consumes, so the
pipeline is unchanged. It also gets identity, multicast discovery, and clock sync
for free. Driven single-threaded from the camera thread (mith's `EntityRegistry`
is not thread-safe).

**Note — two ECS layers coexist by design.** mith is **N=1-per-World** (one self
entity), so it owns *comms / clock / identity / neighbour state*; our multi-
observation **fusion** ECS (aggregator buffers, Kalman bank, fused pose) stays on
the lightweight first-party `mec::ecs` World/scheduler (`include/mec/ecs/world.h`).
That runtime is the app's permanent fusion engine; mith is the coordination
substrate. (The former `mock/mith` shim has been removed.)

**Status:** wired behind the CMake option `MEC_USE_MITH` (default OFF → UDP
stopgap). The submodule is vendored at `third_party/mith-atomas` (v1.0.0-rc1)
and the integration is **compiled and verified**:
- **Linux:** `MithRuntime` compiled + linked against the real headers with zero
  source corrections; the `mith_node_demo` two-node test shows nodes discovering
  each other over multicast (`neighbours=1`), CUSTOM payloads delivered, and the
  synced clock advancing.
- **Android (arm64-v8a):** the NDK cross-compiles `mith-atomas` into `libmith.a`
  and statically links it into `libmec_jni.so` (debug APK builds, ~7 MB native
  lib with `mec::MithRuntime` + `mith::World`/`BeaconSystem` symbols present).

On Android the existing Kotlin `MulticastLock` is required for inbound multicast
delivery.

With a shared frame, the anisotropic fusion (§15.3) triangulates: each phone is
sharp in its image plane and uncertain along its ray, so phones at different
angles pin down each other's depth → real 3D, including for a phone with no line
of sight (through-wall).

**Field-untested:** the comms substrate is verified (Linux two-node discovery +
Android cross-compile/link), but the two-phone *geometry* (and any residual
camera→body extrinsic offset) still needs on-site validation with ≥2 physical
phones on one Wi-Fi. A printed fiducial marker would later give fully automatic
co-localization (position + orientation) without manual pins.

### 15.14 Landscape lock, render-side rotation, and One-Euro smoothing *(feature)*

Three coupled on-device decisions for the live skeleton overlay, each with its
reasoning:

- **Keep MediaPipe input upright; rotate the *render*, not the camera path.**
  BlazePose detects best on an upright person, so `CameraController` continues to
  rotate captured frames to upright before inference (detection quality is the
  priority — "take some performance if needed"). The on-screen overlay was then
  90° off because the `SurfaceView` preview shows the *raw* landscape sensor
  buffer while the landmarks are in the *rotated* (portrait) frame. **Decision:**
  undo the camera's rotation in the renderer only — `OverlayView.rotationDeg =
  (360 − cameraAppliedRotation) % 360` — instead of changing the detection input.
  *Reasoning:* this fixes the visual mismatch with zero risk to the detector, and
  keeps the rotation logic in one place (the view that draws), derived from the
  one source of truth (`CameraController.appliedRotation()`).
- **Lock the activity to `landscape`.** *Reasoning:* the app is a wide-scene
  observer; a fixed orientation removes runtime re-layout/rotation churn and makes
  the preview↔overlay rotation a single constant rather than a per-frame variable.
- **One-Euro filter on the 33 landmarks (x,y,z).** *Reasoning:* partial / low-
  confidence joints were jittery. A plain EMA either lags fast motion or fails to
  kill still-pose jitter; the One-Euro adaptive low-pass (Casiez et al.) does
  both — low cutoff when still (kills jitter), high cutoff when moving (no lag).
  Filters reset on track-loss so re-acquisition doesn't interpolate from a stale
  pose. Visibility is left unfiltered (it gates rendering, it isn't a position).
  Presence confidence was also raised 0.3→0.5 to drop phantom joints, and the
  overlay hides joints below 0.3 visibility.

---

## 16. Testing, Verification & Measured Performance

This section records *how the system is verified* and *why the test surface is
shaped the way it is* — the user-facing companion to the numbers in
[`docs/METRICS_REPORT.md`](docs/METRICS_REPORT.md).

### 16.1 Test harness — decision & reasoning

- **No GTest; a 40-line header (`tests/unit/test_util.h`).** *Reasoning:* §13
  mandates a zero-dependency core that builds anywhere (Linux x86, Android NDK).
  A test main returns non-zero on failure and CTest treats that as fail; `CHECK`
  / `CHECK_NEAR` cover everything the math needs without vendoring a framework.
- **Loopback sockets for the transports, not mocks.** *Reasoning:* the UDP beacon
  and the WebSocket server *are* OS-socket code; mocking the socket layer would
  test the mock. The integration tests bind real ephemeral ports on `127.0.0.1`
  and exchange real datagrams/frames.
- **Synthetic ground truth for accuracy.** *Reasoning:* without two instrumented
  phones there is no real-world 3D ground truth, so accuracy is measured against
  a deterministic synthetic skeleton with controlled per-observer noise
  (`sim::SyntheticObserver`). This isolates the *fusion* error from sensor error.

### 16.2 Coverage (20 CTest cases)

| Area | Tests | Surface covered |
|---|---|---|
| Fusion math | `multi_observer_fusion`, `anisotropic_fusion`, `fusion_edges` | weighted LS, information-form 3×3, zero-weight rejection, covariance output, geometry bonus |
| Temporal | `kalman_tracker` | init, constant-velocity prediction, confidence decay, anisotropic gain, isotropic fallback |
| Observer | `keypoint_projector`, `los_detector`, `lucas_kanade`, `temporal_stereo`, `imu_integrator` | projection, LOS hysteresis, optical flow, depth, strapdown |
| Wire / render | `wire_payload`, `pose_serialiser`, `math` | fixed-point pack/clamp, JSON envelope, vector/quaternion |
| ECS | `ecs_world`, `ecs_pipeline` | store, scheduler rates + topo-order + one-run/tick, full §9 pipeline |
| Transport | `udp_transport`(+`_edges`), `websocket_server`(+`_edges`) | data/presence/probe handshake, magic/self filtering, RFC 6455 handshake, fragmented handshake, 2-client fan-out, extended-length frames |
| Discovery | `device_registry` | manual gating, auto mode, TTL prune, connect-all |

### 16.3 Sanitizer gate — decision & reasoning

The whole suite runs under ASan + LSan + UBSan via the `MEC_SANITIZE` CMake
option (CI job `sanitize`). *Reasoning:* the core manipulates raw sockets,
`memcpy` of packed wire structs, and `std::any` component storage — exactly the
places where leaks/UB hide. `valgrind` is not assumed present (it isn't on the
dev box); compiler sanitizers are portable and CI-friendly. **Result: 20/20
clean, zero leaks, zero UB.**

### 16.4 Live link verification — decision & reasoning

`tools/ws_probe.py` is a dependency-free host client that performs the RFC 6455
handshake by hand and measures the real device→host pose stream (rate, jitter,
keypoint count, confidence). *Reasoning:* the on-device path (Camera2 →
MediaPipe → fusion → WebSocket) cannot be exercised by the C++ unit tests; a
thin host probe over `adb forward` validates the *whole* chain on real hardware
and doubles as the latency/throughput measurement tool for the report.

### 16.5 Benchmark harness

`tools/bench.cpp` (→ `mec_bench`) times the hot ops and the end-to-end fusion
pipeline, and reports fused accuracy vs. synthetic ground truth. It is the
source of the host numbers in `docs/METRICS_REPORT.md` and is reproducible with
a single command.

---

*Document version: 1.1.0 — §15.14 landscape lock + render-side rotation +
One-Euro smoothing; §16 testing/verification with measured metrics; expanded to
20 CTest cases (wire payload, serialiser, math, ECS world, WS/UDP edges, fusion
edges); ASan/LSan/UBSan gate (`MEC_SANITIZE`); `tools/ws_probe.py` device-link
probe + `mec_bench`; full numbers in docs/METRICS_REPORT.md*
*Document version: 1.0.1 — §15.13 real mith-atomas comms integration compiled +
verified: Linux two-node multicast discovery (`mith_node_demo`) and Android
arm64 NDK cross-compile (libmith.a statically linked into libmec_jni.so). Behind
`MEC_USE_MITH` (default OFF → UDP stopgap)*
*Document version: 1.0.0 — first complete end-to-end release: tested C++ core,
Android app (Camera2 + IMU + MediaPipe) on-device, UDP multi-device transport
(§15.10), co-localization via rotation-vector orientation + position pin
(§15.12). "Stable" hardening (CI, accuracy benchmark, Doxygen, API freeze,
field-validated multi-phone) is post-1.0; see §12.*

*Document version: 0.4.1 — UDP multi-device transport (§15.10) + viewer render
upgrade (cylinder bones, smoothing, auto-framing); single-device pose limits
recorded as deferred work (§15.11)*
*Document version: 0.4.0 — Android port scaffolded; debug APK builds (NDK
libmec_jni.so + Camera2/IMU/MediaPipe Kotlin shell, §15.9). On-device run +
real mith-atomas remain*
*Document version: 0.3.9 — §15.3 anisotropic covariance fusion (information form,
view-ray reconstruction in the aggregator); end-to-end ~2.0cm from 12cm-depth-
noise observers. All §15 accuracy items now addressed except v1.0 refinements*
*Document version: 0.3.8 — §15.8 ZUPT (zero-velocity update) bounds IMU velocity
drift; all v0.1+v0.2 algorithms done on Linux — remaining work is the Android
port (capture, MediaPipe, JNI) + the mith-atomas submodule*
*Document version: 0.3.7 — §15.7(a) epipolar subject-motion gating + general
(focus-of-expansion) depth; residual: motion along the epipolar line needs
cross-observer reconciliation*
*Document version: 0.3.6 — §15.7 inter-frame de-rotation implemented (IMUFrame
rotation delta + infinite-homography flow subtraction); subject-motion gating
still open*
*Document version: 0.3.5 — IMUIntegrator strapdown dead-reckoning (§15.8); all
v0.2 observer-pipeline algorithms now done + tested on Linux (Android sampling,
MediaPipe, and JNI bridge remain)*
*Document version: 0.3.4 — TemporalStereoDepth + pyramidal Lucas-Kanade (§15.7,
portable C++17, no OpenCV); v0.2 depth/projection/LOS algorithms done on Linux*
*Document version: 0.3.3 — §9 ECS systems + LOSDetector stubbed against a mock
MithAtomas runtime (§15.6); v0.1 roadmap complete on Linux*
*Document version: 0.3.2 — v0.1 fusion core + hand-rolled WebSocket render
server implemented (§15.5); v0.1 roadmap complete pending MithAtomas ECS wrappers*
*Document version: 0.3.1 — Design review (§15): 128-byte payload fix, double
timestamps, anisotropic-fusion upgrade noted, 33→17 mapping specified*
*Document version: 0.3.0 — Thundercam dependency removed; Camera2 direct capture; self-contained*
*Authors: Vighnesh Patidar*
*Depends on: mith-atomas v1.0+*
*Repository: github.com/Vighnesh-Patidar/myth-eye-cal*
