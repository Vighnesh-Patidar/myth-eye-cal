package com.mythcal.mec

/** JNI surface onto the tested mec_core pipeline (see cpp/mec_jni.cpp). */
object MecNative {
    init { System.loadLibrary("mec_jni") }

    /**
     * Create the pipeline; starts the WS render server + the beacon transport
     * (mith multicast when MEC_USE_MITH, else UDP). `ifaceIp` is the phone's
     * Wi-Fi IPv4 so beacons egress on Wi-Fi rather than the cellular default
     * route; pass "0.0.0.0" to let the kernel choose.
     */
    external fun nativeInit(width: Int, height: Int, port: Int, nodeId: Long,
                            beaconPort: Int, ifaceIp: String): Long

    external fun nativeSetIntrinsics(handle: Long, fx: Float, fy: Float, cx: Float, cy: Float)

    /** Manual co-localization pin: this node's world position (§15.10). */
    external fun nativeSetNodePose(handle: Long, x: Float, y: Float, z: Float)

    /** Absolute device->world orientation (rotation-vector sensor, ENU) (§15.12). */
    external fun nativeSetOrientation(handle: Long, qw: Float, qx: Float, qy: Float, qz: Float)

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
    /** Distinct neighbour phones heard over the beacon recently (§15.10). */
    external fun nativeNeighborCount(handle: Long): Int
    /** Observers fused into the latest pose (local + connected neighbours). */
    external fun nativeObserverCount(handle: Long): Int

    // --- Discovery + manual connect control --------------------------------
    /** Broadcast a discovery probe ("who's there?"); peers reply with presence. */
    external fun nativeScan(handle: Long)
    /**
     * Discovered devices, one per line:
     * "id,ageMs,los,connected(0/1),hasData(0/1),x,y,z". id is an unsigned u64
     * in decimal (parse with toULong()).
     */
    external fun nativeDevices(handle: Long): String
    external fun nativeConnectDevice(handle: Long, id: Long)
    external fun nativeDisconnectDevice(handle: Long, id: Long)
    external fun nativeConnectAll(handle: Long)
    external fun nativeDisconnectAll(handle: Long)
    /** manual=true: only connected devices fuse; false: fuse every visible device. */
    external fun nativeSetManualConnect(handle: Long, manual: Boolean)
    external fun nativeConnectedCount(handle: Long): Int

    external fun nativeShutdown(handle: Long)
}
