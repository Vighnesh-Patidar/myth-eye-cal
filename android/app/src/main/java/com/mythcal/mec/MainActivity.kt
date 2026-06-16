package com.mythcal.mec

import android.Manifest
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.widget.TextView
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import java.net.Inet4Address
import java.net.NetworkInterface
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong

/**
 * Wires the on-device observer pipeline (Camera2 + SensorManager + MediaPipe ->
 * native mec_core -> WebSocket render) and shows a live preview + status. Open
 * the printed ws://<phone-ip>:8080/pose in viewer/myth-eye-cal-viewer.html.
 */
class MainActivity : AppCompatActivity(), CameraController.FrameListener {

    private var handle: Long = 0
    private lateinit var status: TextView
    private lateinit var preview: SurfaceView
    private var camera: CameraController? = null
    private var imu: ImuController? = null
    private var pose: PoseEstimator? = null

    private val width = 640
    private val height = 480
    private val port = 8080
    private val lensPriorM = 2.5f

    private val frames = AtomicLong(0)
    private val lastLandmarks = AtomicInteger(0)
    private var surfaceReady = false
    private var permissionGranted = false
    private var started = false
    private val ui = Handler(Looper.getMainLooper())

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        status = findViewById(R.id.status)
        preview = findViewById(R.id.preview)
        preview.holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) { surfaceReady = true; maybeStart() }
            override fun surfaceChanged(h: SurfaceHolder, f: Int, w: Int, ht: Int) {}
            override fun surfaceDestroyed(holder: SurfaceHolder) { surfaceReady = false }
        })

        permissionGranted = ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED
        if (!permissionGranted) {
            ActivityCompat.requestPermissions(this, arrayOf(Manifest.permission.CAMERA), 1)
        }
        maybeStart()
        ui.post(statusTick)
    }

    override fun onRequestPermissionsResult(rc: Int, perms: Array<out String>, res: IntArray) {
        super.onRequestPermissionsResult(rc, perms, res)
        permissionGranted = res.isNotEmpty() && res[0] == PackageManager.PERMISSION_GRANTED
        if (!permissionGranted) status.text = "Camera permission denied." else maybeStart()
    }

    private fun maybeStart() {
        if (started || !surfaceReady || !permissionGranted) return
        started = true
        handle = MecNative.nativeInit(width, height, port)
        pose = PoseEstimator(this)
        camera = CameraController(this, width, height, this).also { cc ->
            cc.intrinsics().let { MecNative.nativeSetIntrinsics(handle, it.fx, it.fy, it.cx, it.cy) }
            cc.start(preview.holder.surface)
        }
        imu = ImuController(this, handle).also { it.start() }
    }

    // Camera background thread.
    override fun onFrame(yPlane: ByteArray, bitmap: Bitmap, w: Int, h: Int, tsNs: Long) {
        val p = pose ?: return
        val landmarks = p.detect(bitmap)
        lastLandmarks.set(landmarks.size / 4)
        frames.incrementAndGet()
        MecNative.nativeOnFrame(handle, yPlane, w, h, tsNs, landmarks, landmarks.size / 4, lensPriorM)
    }

    private val statusTick = object : Runnable {
        override fun run() {
            if (started && handle != 0L) {
                val ip = localIpv4() ?: "<phone-ip>"
                val clients = MecNative.nativeClientCount(handle)
                val detected = lastLandmarks.get() > 0
                status.text = buildString {
                    append("Myth-Eye-Cal — live\n")
                    append("frames: ${frames.get()}\n")
                    append("pose detected: ${if (detected) "YES" else "no (stand in view)"}\n")
                    append("viewers: $clients\n")
                    append("ws://$ip:$port/pose")
                }
            }
            ui.postDelayed(this, 500)
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        ui.removeCallbacks(statusTick)
        camera?.stop(); imu?.stop(); pose?.close()
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
