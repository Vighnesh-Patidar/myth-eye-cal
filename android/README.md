# Myth-Eye-Cal — Android shell (v0.3 bring-up)

On-device observer pipeline: Camera2 capture + SensorManager IMU + MediaPipe
pose → the tested C++ `mec_core` (temporal-stereo depth, fusion, Kalman) →
WebSocket render to a browser. Single device end-to-end; multi-device fusion
needs the real `mith-atomas` transport (a mock stands in today, §15.6).

## Layout
```
app/src/main/cpp/      JNI bridge (mec_jni.cpp) + NDK CMake (builds mec_core)
app/src/main/java/...   MecNative, MainActivity, CameraController, ImuController, PoseEstimator
app/src/main/assets/    <- put pose_landmarker_lite.task here (NOT bundled)
```

## You must supply
1. **MediaPipe model.** Download into `app/src/main/assets/`:
   ```
   curl -L -o app/src/main/assets/pose_landmarker_lite.task \
     https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/latest/pose_landmarker_lite.task
   ```
2. **Real `mith-atomas`** (later) — replace the `../../../../../mock` include in
   `cpp/CMakeLists.txt` with the submodule for multi-device transport.

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

## Known bring-up shortcuts (upgrade later)
- Pose runs on a **grayscale** (R=G=B=Y) bitmap for simplicity — swap in full
  YUV→RGB for best landmark accuracy.
- `lensPriorM` is a constant; wire it from `CaptureResult.LENS_FOCUS_DISTANCE`.
- Node is pinned at the world origin (single device). Position from GPS / manual
  pin for multi-device.
- ABI is `arm64-v8a` only; add others in `app/build.gradle.kts` if needed.
```
