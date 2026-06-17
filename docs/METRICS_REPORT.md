# Myth-Eye-Cal — Measured Metrics Report

Measured performance, accuracy, and resource metrics for the fusion core and
the on-device Android app. Every number below is **measured**, not estimated;
the method and environment for each is given so the figures are reproducible.
Estimated/target figures (the §7 latency budget) are clearly labelled as such.

- Report date: 2026-06-17
- Core commit: fusion core + expanded test suite (20 ctest cases)
- Reproduce the host numbers: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j && ./build/mec_bench`
- Reproduce the device link: `adb forward tcp:9090 tcp:8080 && python3 tools/ws_probe.py --host 127.0.0.1 --port 9090 --seconds 10`

---

## 1. Environments

| Role | Machine | Details |
|---|---|---|
| Host (core, benchmarks, tests) | Linux x86-64 | Intel Core i5-11260H @ 2.6 GHz, 4 cores visible, GCC 15.2, `-O3 -DNDEBUG` (CMake `Release`) |
| Device (Android app) | A059P phone | arm64-v8a, Android 16, MediaPipe `pose_landmarker_lite.task` |

The fusion core is identical on both; the host figures isolate the math, the
device figures capture the full real-time pipeline (camera → MediaPipe → fusion
→ WebSocket).

---

## 2. Latency

### 2.1 Fusion-core micro-latency (host, measured)

Per-call cost of the hot operations, averaged over 2,000,000 iterations each
(`./build/mec_bench`):

| Operation | Latency | Notes |
|---|---|---|
| `MultiObserverFusion::fuse()` (scalar, 3 obs) | **19.2 ns** | inverse-variance weighted mean, 1 keypoint |
| `MultiObserverFusion::fuse_anisotropic()` (3 obs) | **64.0 ns** | information-form 3×3 solve, 1 keypoint |
| `KeypointKalmanTracker::update()` | **102.3 ns** | coupled 6-state predict+update |
| `KeypointKalmanTracker::predict()` | **28.3 ns** | const, render-rate forward projection |
| `serialise_pose()` (17 keypoints) | **11.5 µs** | hand-rolled JSON, 17 keypoints |

### 2.2 End-to-end fusion pipeline (host, measured)

Full per-frame consumer path — 3 observers × 17 keypoints, scalar fuse +
Kalman update + Kalman predict, 200,000 frames:

| Metric | Value |
|---|---|
| Per-frame compute (fuse+update+predict, 17 kp) | **5.13 µs/frame** |
| Sustained throughput | **195,000 frames/s** |

The fusion math is ~4 orders of magnitude faster than the 33 ms frame interval;
the real-time budget is dominated by capture, inference, and render, not fusion.

### 2.3 Device → host pose link (device, measured)

The phone serves fused poses over a hand-rolled RFC 6455 WebSocket
(`ws://0.0.0.0:8080/pose`). Measured from the host via `tools/ws_probe.py` over
`adb forward` (loopback), 8 s window, one person in view:

| Metric | Value |
|---|---|
| Handshake | HTTP 101 OK (RFC 6455) |
| Pose frame rate | **11.4 Hz** |
| Inter-frame interval mean | **88.8 ms** |
| Inter-frame p50 / p95 | **88.2 / 133.9 ms** |
| Visible keypoints / frame | **17.0 / 17** |
| Mean keypoint confidence | **0.78** |

The 11.4 Hz cadence is set by camera capture + MediaPipe `lite` inference +
overlay on this mid-range device, **not** by the link: over loopback the
WebSocket transit itself is sub-millisecond, and the fusion adds ~5 µs (§2.2).
A faster model tier (`full`/`heavy`) trades frame rate for joint accuracy
(§15.11); only `lite` is currently bundled in the APK.

### 2.4 Capture-to-render budget (target, estimated)

ARCHITECTURE.md §7 sets a **target** of <100 ms capture-to-render (94 ms
budgeted). The measured device cadence (≈88 ms between fused frames with `lite`)
is consistent with that budget. A full instrumented capture-to-photon
measurement on two physical phones is still pending (§15.13 field test).

---

## 3. Accuracy

### 3.1 Fused 3D accuracy (host, measured)

Against synthetic ground truth (a walking 17-keypoint skeleton), with three
noisy observers (per-observer isotropic Gaussian noise 3–5 cm), measured by
`mec_bench` and asserted by `tests/integration/test_ecs_pipeline.cpp`:

| Configuration | Mean error | Max error |
|---|---|---|
| 3 isotropic observers (3–5 cm noise) → fuse + Kalman | **2.42 cm** | **9.04 cm** |
| 3 orthogonal-view observers, 12 cm depth noise → anisotropic fuse + Kalman | **< 6 cm** (test gate) | < 20 cm (test gate) |

Fusion + temporal filtering bring the fused error **below the best single
observer's noise**: the geometry bonus (orthogonal views constrain each other's
depth, §5.2/§15.3) plus inverse-variance averaging (√N shrinkage) plus the
Kalman tracker. Full methodology: [`ACCURACY_METHODOLOGY.md`](ACCURACY_METHODOLOGY.md).

### 3.2 Wire-format precision (measured)

The 128-byte `KeypointFramePayload` packs world coordinates as fixed-point cm:

| Property | Value |
|---|---|
| Spatial quantisation | 1 cm grid, error ≤ 0.5 cm (verified, `test_wire_payload`) |
| Coordinate range | ±327.67 m (saturating, no wraparound) |
| Confidence quantisation | 8-bit, [0,1] → {0..255}, monotonic |

This 0.5 cm wire error is well below the ~2.4 cm fused accuracy, so the payload
quantisation is not the accuracy bottleneck.

### 3.3 On-device detection robustness (device, measured)

With the One-Euro smoothing + tuned confidences (§15.14): 17/17 keypoints
reported per frame at 0.78 mean confidence for a single person in view (§2.3).

---

## 4. Memory

### 4.1 Device runtime memory (measured, `adb shell dumpsys meminfo`)

App running, camera + MediaPipe `lite` + WebSocket server active:

| Bucket | PSS |
|---|---|
| **Total PSS** | **266 MB** (272,394 KB) |
| Total RSS | 379 MB |
| Native heap | 143 MB |
| Graphics | 42 MB |
| Dalvik heap | 20 MB |

Native heap dominates (camera buffers + MediaPipe tensors). This is comfortably
within budget for a modern phone; the `lite` model keeps the inference footprint
small relative to `full`/`heavy`.

### 4.2 Fusion-core footprint (host, measured)

| Artifact | Size |
|---|---|
| `libmec_core.a` (Release static lib) | 73 KB |
| Core source (include + src) | 3,096 LOC |

The core has zero third-party dependencies; the binary footprint is negligible.

### 4.3 Leak / undefined-behaviour audit (host, measured)

The **entire** 20-case test suite was run under AddressSanitizer +
LeakSanitizer + UndefinedBehaviorSanitizer:

```
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DMEC_SANITIZE=address,undefined
cmake --build build-asan -j
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
ctest --test-dir build-asan --output-on-failure
```

**Result: 20/20 passed, zero leaks, zero UB reported.** This covers the socket
lifecycles (UDP transport, WebSocket server accept/close/reap), the ECS
component store (`std::any` lifetimes), and all fusion/Kalman math.

---

## 5. Compute / power proxy

| Metric | Value | Source |
|---|---|---|
| App CPU (device, steady state) | ~106% of one core (≈1.06 cores; ~13% of 8 cores) | `adb shell top` |

Dominated by MediaPipe inference + YUV→RGB conversion. The fusion + transport
adds negligible CPU (µs-scale per frame, §2).

---

## 6. Artifact sizes

| Artifact | Size |
|---|---|
| Debug APK | 27.8 MB |
| Bundled pose model (`pose_landmarker_lite.task`) | 5.78 MB |
| `libmec_core.a` | 73 KB |

---

## 7. Test coverage summary

| Suite | Cases | What |
|---|---|---|
| Unit | 14 | fusion (scalar/aniso/edges), Kalman, projector, LOS, Lucas-Kanade, temporal stereo, IMU, device registry, wire payload, pose serialiser, math, ECS world/scheduler |
| Integration | 6 | fusion pipeline, ECS pipeline, WebSocket (happy + edges), UDP transport (handshake + edges) |
| **Total** | **20** | run via `ctest`; ~0.34 s (Release), ~0.98 s (ASan) |

Plus the live device↔host link verification (`tools/ws_probe.py`) and the
benchmark/accuracy harness (`tools/bench.cpp` → `mec_bench`).

---

## 8. Known gaps (honest accounting)

- **Two-phone geometry** (the cross-device triangulation that yields through-wall
  3D) is verified at the comms layer (Linux two-node multicast, Android
  cross-compile/link) but not yet field-measured with ≥2 physical phones
  (§15.13). All multi-observer accuracy numbers above are from the synthetic
  harness.
- **Capture-to-photon latency** on device is inferred from the frame cadence,
  not instrumented end to end.
- Only the `lite` pose model is bundled; `full`/`heavy` would change §2.3/§3.3
  (higher accuracy, lower frame rate).
