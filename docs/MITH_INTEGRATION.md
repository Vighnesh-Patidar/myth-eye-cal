# MithAtomas Integration — contract & port plan

Replaces the mock (`mock/mith/atomas.h`) + UDP stopgap (§15.10) with the real
`mith-atomas` runtime. Our dependency surface is small and isolated; this is the
exact contract the real API must satisfy (or that we adapt our code to).

## 1. Our dependency surface (everything we use from `mith::`)

| Symbol | How we use it | Files |
|---|---|---|
| `World` | entity/component store: `create_entity()`, `local()`/`set_local()`, `now_s()`, `add<C>()`, `get<C>()`, `get_or_add<C>()` | `mec_jni.cpp`, all systems |
| `System` | base class; `update(World&, double dt)` | `systems/*` |
| `SystemScheduler` | `add(system, after-deps)`, `tick(world, dt)` | `mec_jni.cpp` |
| `ColdComponent<T>` / `HotComponent<T>` | CRTP component base tags | `components/*` |
| `PositionComponent {x,y,z}` | node world position | broadcast/projection |
| `OrientationComponent {qw,qx,qy,qz}` | node orientation | (projection) |
| `BehaviourStateComponent {los_state}` | this node's LOS, replicated | observer activation |
| `UserStateVector {sender, los_state, recv_time_s, spx/spy/spz, payload[128]}` | one neighbour beacon | aggregator, `mec_jni` |
| `UserNeighbourTable {entries}` | received neighbour beacons | aggregator |
| `UserBeaconChannel.broadcast(payload[128])` | send our beacon | `mec_jni`, broadcast system |
| `EntityId`, `NodeId` | ids | everywhere |

The keypoint payload is exactly 128 bytes (`KeypointFramePayload`, §4.5).

## 2. What the real mith-atomas must tell us (open questions)

**Build / consumption**
- How is it consumed — CMake target (name?), header-only, static, or shared?
- Does it build for **Android NDK (arm64-v8a)**, or Linux/desktop only? (Decides
  whether Android uses real mith or keeps the UDP shim while desktop nodes use mith.)
- External deps (libsodium for Ed25519? asio? etc.) and their NDK availability.

**ECS**
- Real entity/component store API (equivalents of `add/get/create_entity`).
- Component declaration: real base(s) for hot vs cold; how a component is registered.
- `System` base signature + how systems register with the scheduler and declare
  order/rate; **who owns the run loop** (mith ticks systems, or we call it?).

**Comms (the core)**
- User beacon channel: how to **broadcast** a payload, the **real max payload size**,
  and how to **enumerate neighbour beacons** with each neighbour's node id,
  `BehaviourState` (LOS), and `Position`.
- Clock sync: the synced-time API (our `now_s()`).

**Bootstrap**
- Node startup/join: identity (Ed25519 key?), peer **discovery** (mDNS/broadcast?),
  and config (network interface, ports).
- **Threading model**: does mith run its own threads/event loop? How do we feed
  per-frame observations in and read neighbour data out, given our camera runs on
  a background `HandlerThread`?

## 3. Port plan (once the API is known)
1. Drop in real mith (submodule + CMake), delete `mock/mith/`.
2. Either (a) write a thin `mith` adapter mapping our small surface onto the real
   API, or (b) adjust our `systems/*` + `mec_jni` to mith's real System/scheduler
   and beacon-channel calls — pick based on how close the shapes are.
3. Replace `UdpBeaconTransport` usage in `mec_jni` with the real user beacon
   channel; keep UDP only if Android can't run real mith.
4. Re-verify: Linux node tests + on-device run; then multi-device through-wall.

## 4. Implementation status (resolved against the real API)
`MithRuntime` (`transport/mith_runtime.{h,cpp}`) is written against the real API:
`mith::World` + `UDPMulticastTransport::open` + `BeaconSystem`/`DiscoverySystem`/
`ClockSyncSystem`; self `Position`/`BehaviourState` set on the registry; keypoint
frames sent as `CUSTOM` `Message`s; received via `CommBufferComponent.queue` and
paired with `neighbour_table()` entries. It mirrors `UdpBeaconTransport`
(`broadcast` + `poll() -> BeaconObservation`) so the fusion path is unchanged.

mith is **N=1-per-World**, so it provides comms/clock/identity/neighbours; our
multi-observation fusion stays on the `mock/mith` World/scheduler (now its
permanent role, not a transport stand-in).

## 5. How to enable
```sh
git submodule add <mith-atomas-url> third_party/mith-atomas
git submodule update --init --recursive
```
Android (`app/build.gradle.kts` → `externalNativeBuild.cmake`):
```kotlin
arguments += "-DMEC_USE_MITH=ON"
```
Then rebuild. With the flag OFF (default) the UDP stopgap is used (builds + runs
today). `MithRuntime` is unbuilt until the submodule is present — expect a few
field-name fixes on first compile against the real headers (e.g. exact
`WorldConfig` / `UDPMulticastTransport::Config` members, `neighbour_table`
entry fields, `HierarchicalID` comparison).

