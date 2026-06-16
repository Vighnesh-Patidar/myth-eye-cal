package com.mythcal.mec

/** JNI surface onto the tested mec_core pipeline (see cpp/mec_jni.cpp). */
object MecNative {
    init { System.loadLibrary("mec_jni") }

    /** Create the pipeline; starts the WS render server + UDP beacon transport. */
    external fun nativeInit(width: Int, height: Int, port: Int, nodeId: Long, beaconPort: Int): Long

    external fun nativeSetIntrinsics(handle: Long, fx: Float, fy: Float, cx: Float, cy: Float)

    /** Manual co-localization pin: this node's world position (§15.10). */
    external fun nativeSetNodePose(handle: Long, x: Float, y: Float, z: Float)

    /** One IMU sample (specific force incl. gravity, rad/s gyro, dt seconds). */
    external fun nativeOnImuSample(
        handle: Long, ax: Float, ay: Float, az: Float,
        gx: Float, gy: Float, gz: Float, dt: Float
    )

    /**
     * One camera frame + its MediaPipe landmarks. landmarks is float[count*4] =
     * (x_norm, y_norm, z, visibility). Returns the broadcast pose_frame JSON.
     */
    external fun nativeOnFrame(
        handle: Long, yPlane: ByteArray, w: Int, h: Int, tsNs: Long,
        landmarks: FloatArray, count: Int, lensPriorM: Float
    ): String

    external fun nativePoll(handle: Long)
    external fun nativeServerPort(handle: Long): Int
    external fun nativeClientCount(handle: Long): Int
    external fun nativeShutdown(handle: Long)
}
