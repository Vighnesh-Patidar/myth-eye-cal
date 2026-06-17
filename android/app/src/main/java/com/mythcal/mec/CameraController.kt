package com.mythcal.mec

import android.content.Context
import android.graphics.Bitmap
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.media.Image
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.view.Surface

/**
 * Camera2 capture (§4.1) — no intermediate framework. Streams YUV_420_888 at a
 * fixed sensor size, then per frame:
 *  - converts YUV -> RGB (MediaPipe pose needs colour, not the old grayscale
 *    bring-up shortcut), and
 *  - rotates to UPRIGHT using the sensor orientation, so a phone held in
 *    portrait feeds an upright body to the pose tracker (BlazePose fails on
 *    sideways frames).
 *
 * The Y (luma) plane handed to native (temporal-stereo depth / LK) is the SAME
 * upright frame, and intrinsics() reports the upright pinhole — so landmarks,
 * depth and projection all share one coordinate frame.
 */
class CameraController(
    private val context: Context,
    /** Sensor capture size (landscape). Output is rotated to upright. */
    private val captureW: Int = 640,
    private val captureH: Int = 480,
    private val listener: FrameListener
) {
    interface FrameListener {
        fun onFrame(yPlane: ByteArray, bitmap: Bitmap, w: Int, h: Int, tsNs: Long)
    }

    data class Intrinsics(val fx: Float, val fy: Float, val cx: Float, val cy: Float)

    private val cm = context.getSystemService(Context.CAMERA_SERVICE) as CameraManager
    private var device: CameraDevice? = null
    private var reader: ImageReader? = null
    private var thread: HandlerThread? = null
    private var handler: Handler? = null
    private var cameraId: String = ""

    // Reused per-frame scratch buffers (camera thread only).
    private var argb = IntArray(0)
    private var yOut = ByteArray(0)

    /** Sensor orientation (0/90/180/270); rotation applied to reach upright. */
    private val rotation: Int by lazy {
        cm.getCameraCharacteristics(backCameraId())
            .get(CameraCharacteristics.SENSOR_ORIENTATION) ?: 90
    }
    private val swaps get() = rotation == 90 || rotation == 270

    /** Rotation (deg, CW) applied to reach upright — the renderer undoes this. */
    fun appliedRotation(): Int = rotation

    /** Upright output dimensions (axes swap for 90/270). */
    fun outputWidth(): Int = if (swaps) captureH else captureW
    fun outputHeight(): Int = if (swaps) captureW else captureH

    /** Pinhole intrinsics for the UPRIGHT output frame. */
    fun intrinsics(): Intrinsics {
        val ch = cm.getCameraCharacteristics(backCameraId())
        val focalMm = ch.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull() ?: 4.0f
        val physical = ch.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
        val sensorWmm = physical?.width ?: 5.0f
        val sensorHmm = physical?.height ?: 3.75f
        val oW = outputWidth(); val oH = outputHeight()
        // After a 90/270 rotation the output x-axis spans the sensor's height.
        return if (swaps) {
            Intrinsics(focalMm / sensorHmm * oW, focalMm / sensorWmm * oH, oW / 2f, oH / 2f)
        } else {
            Intrinsics(focalMm / sensorWmm * oW, focalMm / sensorHmm * oH, oW / 2f, oH / 2f)
        }
    }

    /** @param previewSurface optional on-screen preview target (SurfaceView). */
    @Suppress("MissingPermission") // caller must hold CAMERA permission
    fun start(previewSurface: Surface? = null) {
        thread = HandlerThread("mec-camera").also { it.start() }
        handler = Handler(thread!!.looper)
        cameraId = backCameraId()

        reader = ImageReader.newInstance(captureW, captureH, ImageFormat.YUV_420_888, 2).apply {
            setOnImageAvailableListener({ r ->
                val image = r.acquireLatestImage() ?: return@setOnImageAvailableListener
                try {
                    process(image)
                } finally {
                    image.close()
                }
            }, handler)
        }

        cm.openCamera(cameraId, object : CameraDevice.StateCallback() {
            override fun onOpened(camera: CameraDevice) {
                device = camera
                val targets = ArrayList<Surface>()
                previewSurface?.let { targets.add(it) }
                targets.add(reader!!.surface)
                val req = camera.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW).apply {
                    targets.forEach { addTarget(it) }
                    set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO)
                }
                @Suppress("DEPRECATION")
                camera.createCaptureSession(targets,
                    object : android.hardware.camera2.CameraCaptureSession.StateCallback() {
                        override fun onConfigured(session: android.hardware.camera2.CameraCaptureSession) {
                            session.setRepeatingRequest(req.build(), null, handler)
                        }
                        override fun onConfigureFailed(session: android.hardware.camera2.CameraCaptureSession) {}
                    }, handler)
            }
            override fun onDisconnected(camera: CameraDevice) { camera.close(); device = null }
            override fun onError(camera: CameraDevice, error: Int) { camera.close(); device = null }
        }, handler)
    }

    fun stop() {
        device?.close(); device = null
        reader?.close(); reader = null
        thread?.quitSafely(); thread = null; handler = null
    }

    private fun backCameraId(): String {
        for (id in cm.cameraIdList) {
            val facing = cm.getCameraCharacteristics(id).get(CameraCharacteristics.LENS_FACING)
            if (facing == CameraCharacteristics.LENS_FACING_BACK) return id
        }
        return cm.cameraIdList.firstOrNull() ?: ""
    }

    // YUV_420_888 -> upright RGB bitmap + upright Y plane, single pass.
    private fun process(image: Image) {
        val W = captureW; val H = captureH
        val oW = outputWidth(); val oH = outputHeight()
        if (argb.size != oW * oH) { argb = IntArray(oW * oH); yOut = ByteArray(oW * oH) }

        val yp = image.planes[0]; val up = image.planes[1]; val vp = image.planes[2]
        val yb = yp.buffer; val ub = up.buffer; val vb = vp.buffer
        val yArr = ByteArray(yb.remaining()); yb.get(yArr)
        val uArr = ByteArray(ub.remaining()); ub.get(uArr)
        val vArr = ByteArray(vb.remaining()); vb.get(vArr)
        val yRow = yp.rowStride
        val uRow = up.rowStride; val uPix = up.pixelStride
        val vRow = vp.rowStride; val vPix = vp.pixelStride
        val rot = rotation

        for (sy in 0 until H) {
            val uvRow = sy shr 1
            for (sx in 0 until W) {
                val Y = yArr[sy * yRow + sx].toInt() and 0xFF
                val uvCol = sx shr 1
                val U = (uArr[uvRow * uRow + uvCol * uPix].toInt() and 0xFF) - 128
                val V = (vArr[uvRow * vRow + uvCol * vPix].toInt() and 0xFF) - 128
                var r = (Y + 1.370705f * V).toInt()
                var g = (Y - 0.337633f * U - 0.698001f * V).toInt()
                var b = (Y + 1.732446f * U).toInt()
                if (r < 0) r = 0 else if (r > 255) r = 255
                if (g < 0) g = 0 else if (g > 255) g = 255
                if (b < 0) b = 0 else if (b > 255) b = 255

                val dx: Int; val dy: Int
                when (rot) {
                    90 -> { dx = H - 1 - sy; dy = sx }
                    180 -> { dx = W - 1 - sx; dy = H - 1 - sy }
                    270 -> { dx = sy; dy = W - 1 - sx }
                    else -> { dx = sx; dy = sy }
                }
                val di = dy * oW + dx
                argb[di] = (0xFF shl 24) or (r shl 16) or (g shl 8) or b
                yOut[di] = Y.toByte()
            }
        }

        val bmp = Bitmap.createBitmap(argb, oW, oH, Bitmap.Config.ARGB_8888)
        listener.onFrame(yOut, bmp, oW, oH, image.timestamp)
    }
}
