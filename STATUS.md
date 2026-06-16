# Myth-Eye-Cal — Status & v0.3 Hand-off

**As of:** 2026-06-16 · **Design doc:** [ARCHITECTURE.md](ARCHITECTURE.md) v0.3.5

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
| ECS §8/§9 | components + all 6 `System`s + mock runtime | `components/`, `systems/`, `mock/mith/atomas.h` | `test_ecs_pipeline` |
| Sim §12 | synthetic LOS-node generators | `sim/synthetic_pose.h`, `sim/beacon_pack.h` | `test_fusion_pipeline`, `test_ecs_pipeline` |

**Runnable demos** (`build/`): `sim_pose_demo` (fusion→JSON to stdout),
`render_server_demo` (fusion→WebSocket), `ecs_pipeline_demo` (full §9 scheduler
→ WebSocket). Each verified live: real browser-style client receives
`pose_frame` text frames. *(Note: connecting a client across two separately
sandboxed processes fails on network isolation — run server and client in one
shell, or just trust `test_websocket_server`'s in-process loopback.)*

## Blocked — needs Android/NDK or the real submodule

| Item | Blocker | Roadmap |
|---|---|---|
| MediaPipe Pose Landmarker / MoveNet | Android AAR + GPU delegate | v0.2 |
| `Camera2` capture + `FrameMetaBuilder` | Android Camera2 API | v0.2 |
| `SensorManager` 200 Hz sampling | Android sensors (math is done — feeds `IMUIntegrator`) | v0.2 |
| Android JNI bridge / `.so` via NDK | Android NDK | v0.2 |
| Real `mith::` runtime | `mith-atomas` submodule (mock stands in — `mock/mith/`) | all |
| Native OpenGL ES renderer | Android (browser path works today) | v0.2/v0.3 |
| End-to-end multi-phone + latency runs | physical devices | v0.3 |

The interfaces these plug into already exist and are exercised by tests:
`PoseEstimatorBackend` (pose), `Frame`/`IMUFrame` (capture/IMU),
`LatestObservationComponent` (observer→ECS seam), and the `mith::` mock.

## Open design items (carry forward)

From ARCHITECTURE.md §15. **Fixed** = already corrected in code + doc;
**Upgrade** = known limitation with a documented plan.

- §15.1 **Fixed** — 128-byte payload overflow (was 147 B).
- §15.2 **Fixed** — `double` timestamps (float couldn't hold a synced clock).
- §15.3 **Upgrade** — scalar isotropic `uncertainty_r` can't model anisotropic
  monocular depth error; wants a per-observation 3×3 covariance.
- §15.4 **Fixed** — 33→17 keypoint mapping specified (`kMediapipe33To17`).
- §15.5 **Deviation** — hand-rolled RFC 6455 server instead of uWebSockets.
- §15.6 **Scaffolding** — ECS on a mock `mith::` runtime; swap for the real
  submodule. Also: the wire payload carries no per-observation σ (no room), so
  the aggregator reconstructs σ ≈ 0.03/confidence — lossy.
- §15.7 **Upgrade** — temporal stereo: portable LK (NEON deferred) **and** a
  physical caveat — disparity is contaminated by subject motion + camera
  rotation; needs inter-frame de-rotation (extend `IMUFrame`) and motion gating.
- §15.8 **Upgrade** — IMU velocity drift is unbounded without ZUPT / visual
  velocity fusion ("reset each frame" = reset displacement, not velocity).

Highest-value accuracy work, in order: **§15.7** (de-rotation — biggest depth
error source), **§15.8** (ZUPT), **§15.3** (anisotropic fusion).

## Suggested v0.3 next steps

1. **Vendor `mith-atomas`** and replace `mock/mith/` — the `mith::` names are
   the only coupling; `mec_core` is already free of the mock.
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
mock/mith/atomas.h                  mock MithAtomas (swap for real submodule)
examples/                           sim_pose_demo, render_server_demo, ecs_pipeline_demo
viewer/myth-eye-cal-viewer.html     Three.js browser renderer
tests/{unit,integration}/           10 suites via CTest
```

## Conventions for the next session

- **Commits:** do not auto-commit — surface ready-to-paste commands and commit
  frequently; **no attribution trailer** in commit messages.
- **Critique before implementing:** flag upgrades/critical problems, update
  ARCHITECTURE.md (§15), then proceed.
- `mec_core` stays dependency-free and Linux-testable; Android/mock code lives
  outside it.
