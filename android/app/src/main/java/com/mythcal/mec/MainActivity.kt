package com.mythcal.mec

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.os.Bundle
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.net.Inet4Address
import java.net.NetworkInterface

/**
 * Wires the on-device observer pipeline: Camera2 + SensorManager + MediaPipe ->
 * native mec_core (depth, fusion, WebSocket render). Open the printed
 * ws://<phone-ip>:8080/pose URL in viewer/myth-eye-cal-viewer.html.
 */
class MainActivity : AppCompatActivity(), CameraController.FrameListener {

    private var handle: Long = 0
    private lateinit var status: TextView
    private var camera: CameraController? = null
    private var imu: ImuController? = null
    private var pose: PoseEstimator? = null

    private val width = 640
    private val height = 480
    private val port = 8080
    private val lensPriorM = 2.5f // replaced per-frame by autofocus distance when wired

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        status = findViewById(R.id.status)
        status.text = "Starting…"

        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA)
            != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), 1)
        } else {
            startPipeline()
        }
    }

    override fun onRequestPermissionsResult(rc: Int, perms: Array<out String>, res: IntArray) {
        super.onRequestPermissionsResult(rc, perms, res)
        if (res.isNotEmpty() && res[0] == PackageManager.PERMISSION_GRANTED) startPipeline()
        else status.text = "Camera permission denied."
    }

    private fun startPipeline() {
        handle = MecNative.nativeInit(width, height, port)
        pose = PoseEstimator(this)
        camera = CameraController(this, width, height, this)
        camera!!.intrinsics().let { MecNative.nativeSetIntrinsics(handle, it.fx, it.fy, it.cx, it.cy) }
        imu = ImuController(this, handle).also { it.start() }
        camera!!.start()

        val ip = localIpv4() ?: "<phone-ip>"
        runOnUiThread { status.text = "Live.\nOpen viewer at:\nws://$ip:$port/pose" }
    }

    // Called on the camera background thread for each frame.
    override fun onFrame(yPlane: ByteArray, bitmap: Bitmap, w: Int, h: Int, tsNs: Long) {
        val p = pose ?: return
        val landmarks = p.detect(bitmap)
        MecNative.nativeOnFrame(handle, yPlane, w, h, tsNs, landmarks, landmarks.size / 4, lensPriorM)
    }

    override fun onDestroy() {
        super.onDestroy()
        camera?.stop()
        imu?.stop()
        pose?.close()
        if (handle != 0L) { MecNative.nativeShutdown(handle); handle = 0 }
    }

    private fun localIpv4(): String? {
        for (nif in NetworkInterface.getNetworkInterfaces()) {
            for (addr in nif.inetAddresses) {
                if (!addr.isLoopbackAddress && addr is Inet4Address) return addr.hostAddress
            }
        }
        return null
    }
}
