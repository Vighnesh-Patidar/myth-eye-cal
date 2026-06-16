# Myth-Eye-Cal

Distributed pose reconstruction across a self-organising network of commodity
smartphones — render a person's live 3D pose from devices that have line of
sight, on devices that don't. See [ARCHITECTURE.md](ARCHITECTURE.md) for the
full design.

## Status

**v0.1 — Fusion Core (no camera): in progress.** The pure-geometry / pure-math
fusion core builds and is tested on Linux x86 with zero Android, MithAtomas,
or OpenCV dependencies (per ARCHITECTURE.md §13).

Implemented:

- `MultiObserverFusion` — weighted least-squares fusion of N observers (§5.2)
- `KeypointKalmanTracker` — constant-velocity per-keypoint Kalman tracking (§5.3)
- `KeypointProjector` — image + depth → world-frame projection (§4.4)
- Core wire types incl. the 128-byte `KeypointFramePayload` (§4.5)
- Synthetic LOS-node simulator + `sim_pose_demo` (§12)
- `WebSocketRenderServer` — hand-rolled RFC 6455 server (§6, §15.5)
- `myth-eye-cal-viewer.html` — Three.js skeletal renderer (§6.4)
- `render_server_demo` — live fusion sim served over WebSocket to the browser
- Unit + integration tests (fusion math, Kalman, projector, full pipeline,
  WebSocket handshake + loopback broadcast)

Deferred (need the `mith-atomas` submodule / Android): the ECS `System`
classes (§9), MediaPipe pose, temporal-stereo depth.

## Build & test (Linux)

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

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
include/mec/     core headers (math, types, fusion/, observer/, render/, sim/)
src/             fusion + observer implementations
examples/        sim_pose_demo
viewer/          Three.js browser renderer
tests/           unit/ + integration/
```

## License

Apache 2.0 (see ARCHITECTURE.md §10; `LICENSE` to be added).
