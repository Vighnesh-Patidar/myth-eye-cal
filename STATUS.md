# Myth-Eye-Cal — Status & v0.3 Hand-off

**As of:** 2026-06-16 · **Design doc:** [ARCHITECTURE.md](ARCHITECTURE.md) v1.0.1

This is the map for the next session. It records what is **built and tested on
Linux**, what is **blocked on the Android/NDK toolchain or the real
`mith-atomas` submodule**, and the open design items carried forward.

## TL;DR

Every **algorithm** in the v0.1 fusion/render core and the v0.2 observer
pipeline is implemented as portable C++17 and tested on Linux x86 — **10/10
test suites pass, zero warnings**. What remains for a running phone build is
**platform glue** (camera/IMU capture, MediaPipe, JNI, the coordination layer),
none of which can be built or tested in this environment.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
ctest --test-dir build --output-on-failure        # 10/10 pass
```

Accuracy sanity: 3 synthetic observers at 3–5 cm noise fuse to **~2.5–2.8 cm
mean keypoint error** (standalone and through the ECS scheduler).

## Done — implemented and tested on Linux

| Area | Component | Files | Tests |
|---|---|---|---|
| Fusion §5.2 | `MultiObserverFusion` (weighted LS) | `fusion/multi_observer_fusion.*` | `test_multi_observer_fusion` |
| Fusion §5.3 | `KeypointKalmanTracker` (CV per keypoint) | `fusion/keypoint_kalman.*` | `test_kalman_tracker` |
| Observer §4.4 | `KeypointProjector` (image+depth→world) | `observer/keypoint_projector.*` | `test_keypoint_projector` |
| Observer §3.2 | `LOSDetector` (hysteresis state machine) | `observer/los_detector.h` | `test_los_detector` |
| Observer §4.3 | `LucasKanade` (pyramidal optical flow) | `observer/lucas_kanade.*` | `test_lucas_kanade` |
| Observer §4.3 | `TemporalStereoDepth` (IMU baseline + lens fuse) | `observer/temporal_stereo_depth.*` | `test_temporal_stereo` |
| Observer §4.3 | `IMUIntegrator` (strapdown dead-reckoning) | `observer/imu_integrator.*` | `test_imu_integrator` |
| Wire §4.5 | 128-byte `KeypointFramePayload` + pack/unpack | `types.h` | (asserted at compile time) |
| Render §6.3 | `PoseSerialiser` (pose_frame JSON) | `render/pose_serialiser.h` | covered by pipeline tests |
| Render §6 | `WebSocketRenderServer` (hand-rolled RFC 6455) | `render/websocket_render_server.*` | `test_websocket_server` |
| Render §6.4 | Three.js browser viewer | `viewer/myth-eye-cal-viewer.html` | manual (loads, demo mode) |
| ECS §8/§9 | components + all 6 `System`s + first-party fusion runtime | `components/`, `systems/`, `ecs/world.h` | `test_ecs_pipeline` |
| Sim §12 | synthetic LOS-node generators | `sim/synthetic_pose.h`, `sim/beacon_pack.h` | `test_fusion_pipeline`, `test_ecs_pipeline` |

**Runnable demos** (`build/`): `sim_pose_demo` (fusion→JSON to stdout),
`render_server_demo` (fusion→WebSocket), `ecs_pipeline_demo` (full §9 scheduler
→ WebSocket). Each verified live: real browser-style client receives
`pose_frame` text frames. *(Note: connecting a client across two separately
sandboxed processes fails on network isolation — run server and client in one
shell, or just trust `test_websocket_server`'s in-process loopback.)*

## Blocked — needs Android/NDK or the real submodule

| Item | Status | Roadmap |
|---|---|---|
| Android JNI bridge / `.so` via NDK | **DONE — `libmec_jni.so` builds** (`android/`, §15.9) | v0.2 |
| `Camera2` capture + Kotlin shell | **DONE — compiles** (`CameraController`, bring-up shortcuts noted) | v0.2 |
| `SensorManager` 200 Hz sampling | **DONE — compiles** (`ImuController` → `IMUIntegrator`) | v0.2 |
| MediaPipe Pose Landmarker | **DONE — compiles + model bundled** (`PoseEstimator`) | v0.2 |
| Debug APK build | **DONE — builds on Linux toolchain (26 MB)** | v0.3 |
| On-device run (deploy) | pending — needs a phone (VirtualBox USB passthrough or copy APK) | v0.3 |
| Multi-device transport | **DONE (stopgap) — `UdpBeaconTransport`** over Wi-Fi UDP (§15.10); starts on-device | v0.3 |
| Discovery + manual connect | **DONE — `DeviceRegistry`** (`test_device_registry`): scan probe / presence header, operator-controlled connect allowlist gates fusion (manual by default); UDP probe/announce + mith neighbour-table enumeration; JNI + Devices UI | v0.3 |
| On-device telemetry | **DONE — `OverlayView`** live skeleton over the preview + status counts; collapsible menu sections | v0.3 |
| Real `mith::` comms (`MithRuntime`) | **DONE — compiled + verified** (§15.13): Linux two-node multicast discovery + Android arm64 NDK cross-compile (libmith.a → libmec_jni.so). Behind `MEC_USE_MITH`; UDP stays default | v0.3 |
| Co-localization (shared world frame) | **OPEN** — multi-phone fusion incoherent without it (§15.10); manual position pin added, orientation alignment unsolved | v0.3 |
| Native OpenGL ES renderer | optional (browser path works; viewer upgraded — cylinder bones/smoothing) | v0.2/v0.3 |
| Multi-phone + latency runs | needs ≥2 phones on one Wi-Fi + co-localization | v0.3 |
| Single-device pose quality | **deferred** (§15.11): degenerate depth, gyro drift, grayscale — single-phone is not the target | v0.3+ |

The interfaces these plug into already exist and are exercised by tests:
`PoseEstimatorBackend` (pose), `Frame`/`IMUFrame` (capture/IMU),
`LatestObservationComponent` (observer→ECS seam), and the `mec::` fusion ECS.

## Open design items (carry forward)

From ARCHITECTURE.md §15. **Fixed** = already corrected in code + doc;
**Upgrade** = known limitation with a documented plan.

- §15.1 **Fixed** — 128-byte payload overflow (was 147 B).
- §15.2 **Fixed** — `double` timestamps (float couldn't hold a synced clock).
- §15.3 **Done (incl. residual)** — anisotropic information-form fusion
  (`fuse_anisotropic`, view-ray reconstruction in the aggregator). End-to-end
  ~2.0cm from 12cm-depth-noise observers (`test_anisotropic_fusion`,
  `test_ecs_pipeline`). The former scalar-tracker residual is **resolved**: the
  per-keypoint tracker is now a coupled 6-state Kalman filter taking the full
  fused 3×3 covariance as `R`, so anisotropy carries through time
  (`test_kalman_tracker`); isotropic inputs reduce exactly to the old per-axis
  filters.
- §15.4 **Fixed** — 33→17 keypoint mapping specified (`kMediapipe33To17`).
- §15.5 **Deviation** — hand-rolled RFC 6455 server instead of uWebSockets.
- §15.6 **Resolved** — the ECS now runs on the first-party `mec` fusion runtime
  (`include/mec/ecs/world.h`); the mock has been removed. Residual: the wire
  payload carries no per-observation σ (no room), so the aggregator reconstructs
  σ ≈ 0.03/confidence — lossy.
- §15.7 **Mostly done** — inter-frame **de-rotation** (b) + **epipolar
  subject-motion gating** (a) implemented and tested (`test_temporal_stereo`);
  depth also generalised to focus-of-expansion motion. Residual: subject motion
  *along* the epipolar line needs cross-observer reconciliation. NEON LK still
  deferred to Android.
- §15.8 **Done** — ZUPT bounds IMU velocity drift at rest (`test_imu_integrator`
  5–6). Residual: rest vs constant-velocity ambiguity → wants a visual-velocity
  cross-check (v1.0).

All §15 accuracy items are now addressed. Remaining are **v1.0 refinements**
(optional, Linux-testable, none block the Android port): cross-observer
reconciliation (§15.7 motion-along-epipolar), and a visual-velocity cross-check
(§15.8 rest vs constant-velocity ambiguity).

## Suggested v0.3 next steps

1. ~~**Vendor `mith-atomas`**~~ **DONE** (§15.13) — submodule vendored at
   `third_party/mith-atomas`; `MithRuntime` backs the comms channel behind
   `MEC_USE_MITH` (Linux + Android arm64 verified). The multi-observation
   fusion ECS is now the first-party `mec` runtime (`include/mec/ecs`) by design
   (mith is N=1-per-World, so it cannot host the many-entity fusion graph); the
   mock has been removed. **Next:** flip `MEC_USE_MITH=ON` as the default once
   two-phone field validation passes.
2. **Android shell**: NDK build of `mec_core` as `.so`, JNI bridge, `Camera2` +
   `SensorManager` wiring into `Frame`/`IMUFrame`, MediaPipe behind
   `PoseEstimatorBackend`.
3. **Accuracy upgrades** §15.7 → §15.8 → §15.3 (all testable on Linux first).
4. **Native GL ES renderer** (browser path already works) for the low-latency
   on-device display.

## Repo layout

```
include/mec/{math,types}.h          core types + wire payload
include/mec/{fusion,observer,render,systems,components,sim}/   headers
src/{fusion,observer,render}/       compiled into mec_core (zero external deps)
include/mec/ecs/world.h             first-party fusion ECS (World/System/scheduler)
examples/                           sim_pose_demo, render_server_demo, ecs_pipeline_demo
viewer/myth-eye-cal-viewer.html     Three.js browser renderer
tests/{unit,integration}/           10 suites via CTest
```

## Conventions for the next session

- **Commits:** do not auto-commit — surface ready-to-paste commands and commit
  frequently; **no attribution trailer** in commit messages.
- **Critique before implementing:** flag upgrades/critical problems, update
  ARCHITECTURE.md (§15), then proceed.
- `mec_core` stays dependency-free and Linux-testable; Android/transport code
  lives outside it.
