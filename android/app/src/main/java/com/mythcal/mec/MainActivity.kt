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
import android.view.Gravity
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.Switch
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
    private lateinit var overlay: OverlayView
    private lateinit var deviceList: LinearLayout
    private var camera: CameraController? = null
    private var imu: ImuController? = null
    private var orient: OrientationController? = null
    private var location: LocationController? = null
    @Volatile private var pose: PoseEstimator? = null
    // Guards the pose estimator while the camera thread runs detect() and the UI
    // thread swaps in a different model (§ user-switchable model).
    private val poseLock = Any()
    @Volatile private var switchingModel = false
    private val prefs by lazy { getSharedPreferences("mec", MODE_PRIVATE) }
    private val modelButtons by lazy {
        linkedMapOf(
            R.id.btnModelHeavy to "pose_landmarker_heavy.task",
            R.id.btnModelFull  to "pose_landmarker_full.task",
            R.id.btnModelLite  to "pose_landmarker_lite.task",
        )
    }

    private val width = 640
    private val height = 480
    private val port = 8080
    private val beaconPort = 8079
    private val lensPriorM = 2.5f
    private var multicastLock: WifiManager.MulticastLock? = null
    private var beaconIface = "0.0.0.0"

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
        overlay = findViewById(R.id.overlay)
        deviceList = findViewById(R.id.deviceList)
        preview.holder.addCallback(object : SurfaceHolder.Callback {
            override fun surfaceCreated(holder: SurfaceHolder) { surfaceReady = true; maybeStart() }
            override fun surfaceChanged(h: SurfaceHolder, f: Int, w: Int, ht: Int) {}
            override fun surfaceDestroyed(holder: SurfaceHolder) { surfaceReady = false }
        })

        // Collapsible section headers (model + devices expanded, co-loc collapsed).
        wireCollapse(R.id.modelHeader, R.id.modelBody, "Pose model", true)
        wireCollapse(R.id.devHeader, R.id.devBody, "Devices", true)
        wireCollapse(R.id.colocHeader, R.id.colocBody, "Co-localization", false)
        wireModelButtons()

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

        // MulticastLock lets the Wi-Fi driver DELIVER inbound multicast/broadcast
        // beacons to us; it does NOT control which interface we SEND on — that's
        // pinned via the Wi-Fi interface IP passed to nativeInit below (§15.10/§15.13).
        val wifi = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        multicastLock = wifi.createMulticastLock("mec-beacon").apply {
            setReferenceCounted(true); acquire()
        }

        @Suppress("HardwareIds")
        val androidId = Settings.Secure.getString(contentResolver, Settings.Secure.ANDROID_ID) ?: "mec"
        val nodeId = (androidId.hashCode().toLong() shl 20) xor (System.nanoTime() and 0xFFFFF)

        pose = try {
            PoseEstimator(this, preferredModelOrder(prefs.getString("pose_model", null)))
        } catch (e: Exception) {
            status.text = "No pose model in assets/ — add pose_landmarker_full.task"
            null
        }
        updateModelSelection(pose?.modelName)
        // Camera rotates to upright; the native pipeline + intrinsics use the
        // upright dimensions so landmarks, depth and projection share a frame.
        val cc = CameraController(this, width, height, this)
        val outW = cc.outputWidth(); val outH = cc.outputHeight()
        // Undo the camera's upright rotation so the overlay lines up with the
        // landscape preview (e.g. sensor 90 -> render -90/270).
        overlay.rotationDeg = (360 - cc.appliedRotation()) % 360
        beaconIface = wifiIpv4()
        handle = MecNative.nativeInit(outW, outH, port, nodeId, beaconPort, beaconIface)
        cc.intrinsics().let { MecNative.nativeSetIntrinsics(handle, it.fx, it.fy, it.cx, it.cy) }
        cc.start(preview.holder.surface)
        camera = cc
        imu = ImuController(this, handle).also { it.start() }
        orient = OrientationController(this, handle).also { it.start() }
        location = LocationController(this).also { if (hasPerm(Manifest.permission.ACCESS_FINE_LOCATION)) it.start() }

        // Manual position pin (§15.12).
        findViewById<Button>(R.id.setPos).setOnClickListener {
            val x = field(R.id.posX); val y = field(R.id.posY); val z = field(R.id.posZ)
            MecNative.nativeSetNodePose(handle, x, y, z)
            currentFocus?.clearFocus()
            toast("position set: (%.2f, %.2f, %.2f)".format(x, y, z))
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

        // Discovery + manual connect control. Manual is the default: you choose
        // which nearby phones fuse into the pose.
        MecNative.nativeSetManualConnect(handle, true)
        findViewById<Button>(R.id.scanBtn).setOnClickListener {
            MecNative.nativeScan(handle); toast("scanning…"); refreshDevices()
        }
        findViewById<Button>(R.id.connectAllBtn).setOnClickListener {
            MecNative.nativeConnectAll(handle); refreshDevices()
        }
        findViewById<Button>(R.id.disconnectAllBtn).setOnClickListener {
            MecNative.nativeDisconnectAll(handle); refreshDevices()
        }
        findViewById<Switch>(R.id.manualSwitch).setOnCheckedChangeListener { _, checked ->
            MecNative.nativeSetManualConnect(handle, checked)
            toast(if (checked) "manual connect: you pick who fuses" else "auto: fusing all visible")
        }
    }

    private fun field(id: Int): Float = findViewById<EditText>(id).text.toString().trim().toFloatOrNull() ?: 0f
    private fun dfield(id: Int): Double = findViewById<EditText>(id).text.toString().trim().toDoubleOrNull() ?: 0.0
    private fun toast(m: String) = Toast.makeText(this, m, Toast.LENGTH_SHORT).show()

    // Camera background thread.
    override fun onFrame(yPlane: ByteArray, bitmap: Bitmap, w: Int, h: Int, tsNs: Long) {
        // Hold the lock across detect so a concurrent model swap can't close the
        // landmarker mid-inference. The swap builds the new model off-lock first,
        // so this only ever waits one frame.
        val landmarks = synchronized(poseLock) {
            val p = pose ?: run { overlay.setLandmarks(FloatArray(0), 0); return }
            p.detect(bitmap, tsNs / 1_000_000L)
        }
        val count = landmarks.size / 4
        lastLandmarks.set(count)
        overlay.setLandmarks(landmarks, count) // live on-device skeleton telemetry
        frames.incrementAndGet()
        MecNative.nativeOnFrame(handle, yPlane, w, h, tsNs, landmarks, count, lensPriorM)
    }

    // --- user-switchable pose model ---------------------------------------

    // Try `chosen` first (if any), then the rest of ALL as graceful fallbacks,
    // so a stale preference (model removed from assets) still starts.
    private fun preferredModelOrder(chosen: String?): List<String> =
        if (chosen != null) listOf(chosen) + PoseEstimator.ALL.filter { it != chosen }
        else PoseEstimator.ALL

    private fun wireModelButtons() {
        val available = PoseEstimator.availableModels(this)
        for ((id, file) in modelButtons) {
            val b = findViewById<Button>(id)
            val present = file in available
            b.isEnabled = present
            b.alpha = if (present) 1f else 0.4f
            b.setOnClickListener { switchModel(file) }
        }
    }

    private fun updateModelSelection(activeFile: String?) {
        for ((id, file) in modelButtons)
            findViewById<Button>(id).isSelected = (file == activeFile)
    }

    // Rebuild the estimator on a background thread (heavy can take ~1s to load),
    // then hot-swap it under poseLock. Persists the choice across launches.
    private fun switchModel(file: String) {
        if (switchingModel || file == pose?.modelName) return
        switchingModel = true
        toast("loading ${PoseEstimator.label(file)}…")
        Thread {
            val next = try {
                PoseEstimator(this, preferredModelOrder(file))
            } catch (e: Exception) {
                runOnUiThread { toast("couldn't load ${PoseEstimator.label(file)}"); switchingModel = false }
                return@Thread
            }
            synchronized(poseLock) { pose?.close(); pose = next }
            prefs.edit().putString("pose_model", next.modelName).apply()
            runOnUiThread {
                updateModelSelection(next.modelName)
                toast("model: ${PoseEstimator.label(next.modelName)}")
                switchingModel = false
            }
        }.start()
    }

    // Tappable section header that expands/collapses its body view.
    private fun wireCollapse(headerId: Int, bodyId: Int, title: String, expanded: Boolean) {
        val header = findViewById<TextView>(headerId)
        val body = findViewById<View>(bodyId)
        fun render(exp: Boolean) {
            body.visibility = if (exp) View.VISIBLE else View.GONE
            header.text = (if (exp) "▾ " else "▸ ") + title
        }
        render(expanded)
        header.setOnClickListener { render(body.visibility != View.VISIBLE) }
    }

    private val losNames = arrayOf("NO", "ACQ", "TRK", "OCC")

    // Rebuild the discovered-devices list from native. Connecting is the
    // operator's call: only connected devices fuse (when Manual is on).
    private fun refreshDevices() {
        if (handle == 0L) return
        deviceList.removeAllViews()
        val raw = MecNative.nativeDevices(handle).trim()
        if (raw.isEmpty()) { deviceList.addView(dimText("no devices — tap Scan")); return }
        for (line in raw.split("\n")) {
            val f = line.split(",")
            if (f.size < 5) continue
            val id = f[0].toULongOrNull() ?: continue
            val ageMs = f[1].toIntOrNull() ?: 0
            val los = f[2].toIntOrNull() ?: 0
            val connected = f[3] == "1"
            val hasData = f[4] == "1"
            deviceList.addView(deviceRow(id, ageMs, los, connected, hasData))
        }
    }

    private fun deviceRow(id: ULong, ageMs: Int, los: Int, connected: Boolean, hasData: Boolean): View {
        val row = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL; gravity = Gravity.CENTER_VERTICAL
        }
        val losN = losNames.getOrElse(los) { "?" }
        val info = TextView(this).apply {
            text = "#${id.toString(16).takeLast(6)}  $losN  ${ageMs}ms${if (hasData) "  pose" else ""}"
            setTextColor(if (connected) 0xFF7fd5a0.toInt() else 0xFFcdd6e0.toInt())
            textSize = 12f; typeface = android.graphics.Typeface.MONOSPACE
            layoutParams = LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1f)
        }
        val btn = Button(this).apply {
            text = if (connected) "Disconnect" else "Connect"
            setOnClickListener {
                if (connected) MecNative.nativeDisconnectDevice(handle, id.toLong())
                else MecNative.nativeConnectDevice(handle, id.toLong())
                refreshDevices()
            }
        }
        row.addView(info); row.addView(btn)
        return row
    }

    private fun dimText(s: String): View = TextView(this).apply {
        text = s; setTextColor(0xFF8a93a0.toInt()); textSize = 12f
        typeface = android.graphics.Typeface.MONOSPACE
    }

    private var tick = 0
    private val statusTick = object : Runnable {
        override fun run() {
            if (started && handle != 0L) {
                val ip = localIpv4() ?: "<phone-ip>"
                val clients = MecNative.nativeClientCount(handle)
                val detected = lastLandmarks.get() > 0
                status.text = buildString {
                    append("Myth-Eye-Cal — live\n")
                    append("frames: ${frames.get()}\n")
                    append("pose model: ${pose?.modelName ?: "—"}\n")
                    append("pose detected: ${if (detected) "YES" else "no (stand in view)"}\n")
                    append("nearby: ${MecNative.nativeNeighborCount(handle)}  " +
                        "connected: ${MecNative.nativeConnectedCount(handle)}  " +
                        "fused: ${MecNative.nativeObserverCount(handle)}\n")
                    append("beacon iface: $beaconIface${if (beaconIface == "0.0.0.0") " (no Wi-Fi!)" else ""}\n")
                    append("viewers: $clients\n")
                    append("ws://$ip:$port/pose")
                }
                findViewById<TextView>(R.id.gpsInfo).text = gpsLine()
                if (tick++ % 3 == 0) refreshDevices() // ~1.5s cadence
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

    // Wi-Fi interface IPv4, so the beacon transport pins multicast/broadcast to
    // Wi-Fi instead of the cellular default route (§15.13). Prefers wlan*/ap*;
    // falls back to "0.0.0.0" (kernel default) if no Wi-Fi address is found.
    private fun wifiIpv4(): String {
        try {
            for (nif in NetworkInterface.getNetworkInterfaces()) {
                if (!nif.isUp || nif.isLoopback) continue
                val name = nif.name.lowercase()
                if (!name.startsWith("wlan") && !name.startsWith("ap")) continue
                for (addr in nif.inetAddresses) {
                    if (!addr.isLoopbackAddress && addr is Inet4Address)
                        return addr.hostAddress ?: continue
                }
            }
        } catch (_: Exception) {}
        return "0.0.0.0"
    }
}
