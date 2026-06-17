# Myth-Eye-Cal — Android shell (v0.3 bring-up)

On-device observer pipeline: Camera2 capture + SensorManager IMU + MediaPipe
pose → the tested C++ `mec_core` (temporal-stereo depth, fusion, Kalman) →
WebSocket render to a browser. Single device end-to-end; multi-device fusion
rides the real `mith-atomas` transport (`MEC_USE_MITH`, on for Android) or the
UDP stopgap. The local fusion ECS is mec's own runtime (`include/mec/ecs`).

## Layout
```
app/src/main/cpp/      JNI bridge (mec_jni.cpp) + NDK CMake (builds mec_core)
app/src/main/java/...   MecNative, MainActivity, CameraController, ImuController, PoseEstimator
app/src/main/assets/    <- put a pose_landmarker_*.task here (NOT bundled)
```

## You must supply
1. **MediaPipe model(s).** Download into `app/src/main/assets/` (the `.task`
   files are git-ignored — they are large binaries, not committed). At least one
   must be present. `PoseEstimator` prefers **heavy → full → lite** (first present
   wins); heavier models recognise the body far more reliably at some frame-rate
   cost. Measured on an A059P (arm64, Android 16): `lite` ≈ 11.4 Hz,
   `heavy` ≈ 9.2 Hz.

   | model | size | APK impact | use |
   |---|---|---|---|
   | `pose_landmarker_lite.task`  | 5.8 MB  | small | fastest, weakest |
   | `pose_landmarker_full.task`  | 9.4 MB  | +9 MB | balanced |
   | `pose_landmarker_heavy.task` | 30.7 MB | +31 MB | best recognition (default when present) |

   Fetch all three (so the app starts on heavy and falls back to lite):
   ```
   BASE=https://storage.googleapis.com/mediapipe-models/pose_landmarker
   for m in heavy full lite; do
     curl -L -o app/src/main/assets/pose_landmarker_$m.task \
       "$BASE/pose_landmarker_$m/float16/latest/pose_landmarker_$m.task"
   done
   ```
   With all three bundled the debug APK is ~68 MB.
2. **`mith-atomas` submodule** — checked out at `third_party/mith-atomas`
   (`git submodule update --init --recursive`). The Android build sets
   `MEC_USE_MITH=ON`, so the real transport is linked into `libmec_jni.so`.

## Build
With Android SDK + NDK + JDK 17 available (Android Studio handles this):
```
cd android
./gradlew :app:assembleDebug        # APK -> app/build/outputs/apk/debug/
```

## Deploy + run
- Phone with **USB debugging** on. `adb install -r app/build/outputs/apk/debug/app-debug.apk`
  (in a VirtualBox VM you must enable USB passthrough first; otherwise copy the
  APK to the host and install from there).
- Launch the app, grant Camera. It prints `ws://<phone-ip>:8080/pose`.
- Open `viewer/myth-eye-cal-viewer.html` (this repo) on any device on the same
  Wi-Fi and enter that URL — the live fused pose renders in the browser.

## Pose recognition (steady body tracking)
- `PoseEstimator` runs MediaPipe in **VIDEO mode** (detect-once-then-track), with
  low detection/presence/tracking confidences — so the body keeps tracking when
  turning away or partly occluded, instead of re-detecting (and dropping) every
  frame. Body pose only; no face mesh.
- `CameraController` feeds an **upright RGB** frame: it does YUV→RGB and rotates
  by the sensor orientation. The Y plane and `intrinsics()` use the same upright
  frame, so landmarks/depth/projection stay consistent.

## Multi-device beacon (mith-atomas)
The Android build links the real `mith-atomas` transport (`MEC_USE_MITH=ON`):
each phone multicasts its keypoint frame on `239.10.20.30:47474` and discovers
neighbours automatically. Two things are required on-device and both are wired:
- **Inbound:** a `WifiManager.MulticastLock` (held while running) so the driver
  delivers multicast to the app.
- **Outbound:** the transport is pinned to the **Wi-Fi interface IPv4**
  (`wifiIpv4()` → `nativeInit`). Without this, Android sends multicast on the
  default route, which is **cellular when mobile data is on**, so beacons never
  reach the LAN. The status overlay shows `beacon iface: <ip>`; if it reads
  `0.0.0.0 (no Wi-Fi!)` the phone has no usable Wi-Fi address.

Checklist if `neighbors:` stays 0 with two phones running:
1. Both phones on the **same Wi-Fi** (same subnet), status shows a `192.168.x.y`
   (not `0.0.0.0`) beacon iface.
2. The AP allows multicast between clients (some routers have "AP isolation" or
   drop multicast — try a phone hotspot or a different AP).
3. `adb logcat | grep mec_jni` shows `mith-atomas runtime up (... via iface ...)`.

(A single phone always renders its own pose locally — `neighbors: 0` is normal.)

## Discovery & manual connect (you control who fuses)
The **Devices** panel (collapsible, bottom of the screen) puts connection in your
hands:
- **Scan** broadcasts a discovery probe (a "who's there?" request header); nearby
  phones reply with presence and appear in the list with a short id, LOS state,
  age, and a `pose` tag if they're sending keypoints. (Over mith the neighbour
  table is surfaced automatically; over UDP idle phones announce presence.)
- Each device has a **Connect/Disconnect** toggle. With **Manual connect** on
  (default), only devices you connect are fused into the pose — everything else
  is heard but ignored. **Conn all / Disc all** are bulk shortcuts; turning Manual
  off fuses every visible device (the old auto behaviour).
- The status line shows `nearby / connected / fused` counts.

## On-device telemetry
You don't need the browser to see what's happening:
- A **skeleton overlay** is drawn live over the camera preview from the actual
  MediaPipe landmarks (the 17-keypoint skeleton fusion uses), coloured by
  per-point visibility — green/steady = good recognition, red/jittery = poor.
  (It's stretched to the view; on a front camera set `OverlayView.mirror`.)
- The **status overlay** shows fps, active pose model, detection state, beacon
  interface, and the nearby/connected/fused counts.
- Menu sections (**Devices**, **Co-localization**) are collapsible — tap a header
  to expand/collapse so the controls stay out of the way of the preview.

## Known bring-up shortcuts (upgrade later)
- `lensPriorM` is a constant; wire it from `CaptureResult.LENS_FOCUS_DISTANCE`.
- Node is pinned at the world origin (single device). Position from GPS / manual
  pin for multi-device.
- ABI is `arm64-v8a` only; add others in `app/build.gradle.kts` if needed.
```
