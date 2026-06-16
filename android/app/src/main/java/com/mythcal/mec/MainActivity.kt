package com.mythcal.mec

import android.Manifest
import android.content.Context
import android.content.pm.PackageManager
import android.graphics.Bitmap
import android.net.wifi.WifiManager
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.provider.Settings
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.widget.Button
import android.widget.EditText
import android.widget.TextView
import android.widget.Toast
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
    private var orient: OrientationController? = null
    private var location: LocationController? = null
    private var pose: PoseEstimator? = null

    private val width = 640
    private val height = 480
    private val port = 8080
    private val beaconPort = 8079
    private val lensPriorM = 2.5f
    private var multicastLock: WifiManager.MulticastLock? = null

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

        permissionGranted = hasPerm(Manifest.permission.CAMERA)
        if (!permissionGranted || !hasPerm(Manifest.permission.ACCESS_FINE_LOCATION)) {
            ActivityCompat.requestPermissions(this, arrayOf(
                Manifest.permission.CAMERA,
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_COARSE_LOCATION), 1)
        }
        maybeStart()
        ui.post(statusTick)
    }

    private fun hasPerm(p: String) =
        ContextCompat.checkSelfPermission(this, p) == PackageManager.PERMISSION_GRANTED

    override fun onRequestPermissionsResult(rc: Int, perms: Array<out String>, res: IntArray) {
        super.onRequestPermissionsResult(rc, perms, res)
        permissionGranted = hasPerm(Manifest.permission.CAMERA)
        if (!permissionGranted) { status.text = "Camera permission denied."; return }
        maybeStart()
        if (hasPerm(Manifest.permission.ACCESS_FINE_LOCATION)) location?.start()
    }

    private fun maybeStart() {
        if (started || !surfaceReady || !permissionGranted) return
        started = true

        // Allow the Wi-Fi driver to deliver UDP broadcast beacons (§15.10).
        val wifi = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        multicastLock = wifi.createMulticastLock("mec-beacon").apply {
            setReferenceCounted(true); acquire()
        }

        @Suppress("HardwareIds")
        val androidId = Settings.Secure.getString(contentResolver, Settings.Secure.ANDROID_ID) ?: "mec"
        val nodeId = (androidId.hashCode().toLong() shl 20) xor (System.nanoTime() and 0xFFFFF)

        handle = MecNative.nativeInit(width, height, port, nodeId, beaconPort)
        pose = PoseEstimator(this)
        camera = CameraController(this, width, height, this).also { cc ->
            cc.intrinsics().let { MecNative.nativeSetIntrinsics(handle, it.fx, it.fy, it.cx, it.cy) }
            cc.start(preview.holder.surface)
        }
        imu = ImuController(this, handle).also { it.start() }
        orient = OrientationController(this, handle).also { it.start() }
        location = LocationController(this).also { if (hasPerm(Manifest.permission.ACCESS_FINE_LOCATION)) it.start() }

        // Manual position pin (§15.12).
        findViewById<Button>(R.id.setPos).setOnClickListener {
            MecNative.nativeSetNodePose(handle, field(R.id.posX), field(R.id.posY), field(R.id.posZ))
        }
        // Absolute GPS origin (set the SAME lat/lon/alt on every phone), then
        // each phone fills its ENU offset from it.
        findViewById<Button>(R.id.setOrigin).setOnClickListener {
            location?.setOrigin(dfield(R.id.oLat), dfield(R.id.oLon), dfield(R.id.oAlt))
            toast("origin set")
        }
        findViewById<Button>(R.id.originHere).setOnClickListener {
            val loc = location
            if (loc?.setOriginHere() == true) {
                findViewById<EditText>(R.id.oLat).setText(String.format("%.7f", loc.lat))
                findViewById<EditText>(R.id.oLon).setText(String.format("%.7f", loc.lon))
                findViewById<EditText>(R.id.oAlt).setText(String.format("%.1f", loc.alt))
                toast("origin = here (±${loc.accuracyM.toInt()}m)")
            } else toast("no GPS fix yet")
        }
        findViewById<Button>(R.id.gpsFill).setOnClickListener {
            val enu = location?.enu()
            if (enu == null) toast(if (location?.hasOrigin != true) "set origin first" else "no GPS fix")
            else {
                findViewById<EditText>(R.id.posX).setText(String.format("%.2f", enu[0]))
                findViewById<EditText>(R.id.posY).setText(String.format("%.2f", enu[1]))
                findViewById<EditText>(R.id.posZ).setText(String.format("%.2f", enu[2]))
                MecNative.nativeSetNodePose(handle, enu[0], enu[1], enu[2])
                toast("position set from GPS")
            }
        }
    }

    private fun field(id: Int): Float = findViewById<EditText>(id).text.toString().toFloatOrNull() ?: 0f
    private fun dfield(id: Int): Double = findViewById<EditText>(id).text.toString().toDoubleOrNull() ?: 0.0
    private fun toast(m: String) = Toast.makeText(this, m, Toast.LENGTH_SHORT).show()

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
                findViewById<TextView>(R.id.gpsInfo).text = gpsLine()
            }
            ui.postDelayed(this, 500)
        }
    }

    private fun gpsLine(): String {
        val loc = location ?: return "gps: —"
        return when {
            !hasPerm(Manifest.permission.ACCESS_FINE_LOCATION) &&
                !hasPerm(Manifest.permission.ACCESS_COARSE_LOCATION) -> "gps: permission denied"
            !loc.servicesEnabled() -> "gps: location services OFF"
            !loc.hasFix -> "gps: acquiring… (needs sky/Wi-Fi)"
            else -> "gps ${"%.6f".format(loc.lat)}, ${"%.6f".format(loc.lon)} " +
                "±${loc.accuracyM.toInt()}m/${loc.provider}${if (loc.hasOrigin) " [origin set]" else ""}"
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        ui.removeCallbacks(statusTick)
        camera?.stop(); imu?.stop(); orient?.stop(); location?.stop(); pose?.close()
        if (handle != 0L) { MecNative.nativeShutdown(handle); handle = 0 }
        multicastLock?.let { if (it.isHeld) it.release() }; multicastLock = null
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
