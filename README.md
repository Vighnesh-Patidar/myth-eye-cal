# Myth-Eye-Cal

Distributed pose reconstruction across a self-organising network of commodity
smartphones — render a person's live 3D pose from devices that have line of
sight, on devices that don't. See [ARCHITECTURE.md](ARCHITECTURE.md) for the
full design.

## Status

**End-to-end, on real hardware.** The pure-geometry / pure-math fusion core
builds and is tested on Linux x86 with zero Android, mith-atomas, or OpenCV
dependencies (ARCHITECTURE.md §13), and the Android app runs on-device:
Camera2 → MediaPipe pose → fusion → live WebSocket pose stream, verified at
**11.4 Hz / 17 keypoints** from a physical phone (see
[`docs/METRICS_REPORT.md`](docs/METRICS_REPORT.md)).

Implemented:

- `MultiObserverFusion` — weighted least-squares + anisotropic information-form
  fusion of N observers (§5.2, §15.3)
- `KeypointKalmanTracker` — coupled 6-state constant-velocity Kalman tracker
  carrying full 3×3 covariance (§5.3, §15.3)
- `KeypointProjector` — image + depth → world-frame projection (§4.4)
- Core wire types incl. the 128-byte `KeypointFramePayload` (§4.5)
- `WebSocketRenderServer` — hand-rolled RFC 6455 server (§6, §15.5) +
  `myth-eye-cal-viewer.html` Three.js renderer (§6.4)
- `LOSDetector`, `LucasKanade` + `TemporalStereoDepth`, `IMUIntegrator` (§3.2,
  §4.3, §15.7, §15.8)
- All six §9 ECS `System`s on the first-party **mec fusion runtime**
  (`include/mec/ecs/world.h`)
- `UdpBeaconTransport` (stopgap) + `MithRuntime` (real mith-atomas, behind
  `MEC_USE_MITH`) + `DeviceRegistry` discovery & manual connect (§15.10, §15.13)
- Android app: collapsible UI, on-device skeleton overlay with landscape-locked
  render rotation + One-Euro smoothing (§15.14)
- **20 unit + integration tests**, an ASan/LSan/UBSan gate, a device-link probe
  (`tools/ws_probe.py`), and a benchmark/accuracy harness (`mec_bench`)

Pending: two-phone *geometry* field validation (the comms substrate is verified;
the cross-device triangulation needs ≥2 phones on one Wi-Fi — §15.13).

## Build & test (Linux)

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

### Leak / UB check (AddressSanitizer + LeakSanitizer + UBSan)

```sh
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DMEC_SANITIZE=address,undefined
cmake --build build-asan -j
ctest --test-dir build-asan --output-on-failure   # 20/20, zero leaks, zero UB
```

### Benchmarks & accuracy

```sh
./build/mec_bench   # fusion latency, throughput, fused accuracy vs ground truth
```

### Verify a live device → host link

```sh
adb forward tcp:9090 tcp:8080
python3 tools/ws_probe.py --host 127.0.0.1 --port 9090 --seconds 10
```

Reports handshake status, pose frame rate, jitter, keypoint count, and mean
confidence. Numbers and methodology: [`docs/METRICS_REPORT.md`](docs/METRICS_REPORT.md).

## Run the no-camera demo

```sh
# args: <num_observers> <num_frames>
./build/sim_pose_demo 3 50
```

Each line is a `pose_frame` JSON (ARCHITECTURE.md §6.3): synthetic observers
of one moving person are fused and Kalman-smoothed into a single pose.

## Live render over WebSocket

```sh
# args: <port=8080> <num_observers=3> <seconds=0 (0 = forever)>
./build/render_server_demo 8080 3
```

Then open `viewer/myth-eye-cal-viewer.html` in any browser and point the URL
box at `ws://<host>:8080/pose` — the fused pose streams in at 60 Hz. With no
server reachable the viewer falls back to a synthetic walk (demo mode).

The server is a dependency-free RFC 6455 implementation
(`websocket_render_server.{h,cpp}`); see ARCHITECTURE.md §15.5 for why
uWebSockets was not vendored.

## Layout

```
include/mec/     core headers (math, types, fusion/, observer/, render/, sim/, ecs/, transport/, systems/)
src/             fusion + observer + render + transport implementations
android/         Android app (Camera2 + MediaPipe + JNI bridge)
examples/        sim_pose_demo, render_server_demo, ecs_pipeline_demo, mith_node_demo
tools/           bench.cpp (mec_bench), ws_probe.py (device-link probe)
viewer/          Three.js browser renderer
tests/           unit/ + integration/
docs/            ARCHITECTURE deep-dives, METRICS_REPORT, ACCURACY_METHODOLOGY, MITH_INTEGRATION
```

## Metrics

Measured latency, accuracy, memory, and throughput (host + device) are in
[`docs/METRICS_REPORT.md`](docs/METRICS_REPORT.md). Highlights: fused accuracy
~2.4 cm (mean) vs synthetic ground truth, 5.1 µs/frame fusion compute, device
pose stream at 11.4 Hz, ~266 MB device PSS, zero leaks/UB under sanitizers.

## License

Apache 2.0 (see ARCHITECTURE.md §10; `LICENSE` to be added).
