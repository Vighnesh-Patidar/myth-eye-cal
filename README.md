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
- `myth-eye-cal-viewer.html` — Three.js skeletal renderer (§6.4)
- Unit + integration tests (fusion math, Kalman, projector, full pipeline)

Deferred (need the `mith-atomas` submodule / Android): the ECS `System`
classes (§9), MediaPipe pose, temporal-stereo depth, the uWebSockets server.

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

## View it

Open `viewer/myth-eye-cal-viewer.html` in any browser. With no WebSocket
server reachable it shows a synthetic walk (demo mode); point the URL box at a
live `ws://<host>:8080/pose` endpoint once the render server lands.

To pipe the demo into the viewer today, bridge stdout to a WebSocket with any
small relay (e.g. `websocat -s 8080`); the native uWebSockets server is the
next v0.1 task.

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
