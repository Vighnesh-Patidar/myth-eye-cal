package com.mythcal.mec

import android.content.Context
import android.graphics.Bitmap
import android.graphics.ImageFormat
import android.hardware.camera2.CameraCharacteristics
import android.hardware.camera2.CameraDevice
import android.hardware.camera2.CameraManager
import android.hardware.camera2.CaptureRequest
import android.media.ImageReader
import android.os.Handler
import android.os.HandlerThread
import android.view.Surface

/**
 * Camera2 capture (§4.1) — no intermediate framework. Streams YUV_420_888 at a
 * fixed size, hands the caller the Y (luma) plane (for temporal-stereo depth)
 * and a grayscale Bitmap (for MediaPipe), plus capture intrinsics.
 *
 * The grayscale R=G=B=Y bitmap is a bring-up shortcut; swap in a full
 * YUV->RGB conversion for best pose accuracy (see android/README.md).
 */
class CameraController(
    private val context: Context,
    private val width: Int = 640,
    private val height: Int = 480,
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

    /** Pinhole intrinsics for the chosen capture size, from CameraCharacteristics. */
    fun intrinsics(): Intrinsics {
        cameraId = backCameraId()
        val ch = cm.getCameraCharacteristics(cameraId)
        val focalMm = ch.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)?.firstOrNull() ?: 4.0f
        val physical = ch.get(CameraCharacteristics.SENSOR_INFO_PHYSICAL_SIZE)
        val sensorWmm = physical?.width ?: 5.0f
        val sensorHmm = physical?.height ?: 3.75f
        val fx = focalMm / sensorWmm * width
        val fy = focalMm / sensorHmm * height
        return Intrinsics(fx, fy, width / 2f, height / 2f)
    }

    /** @param previewSurface optional on-screen preview target (SurfaceView). */
    @Suppress("MissingPermission") // caller must hold CAMERA permission
    fun start(previewSurface: Surface? = null) {
        thread = HandlerThread("mec-camera").also { it.start() }
        handler = Handler(thread!!.looper)
        cameraId = backCameraId()

        reader = ImageReader.newInstance(width, height, ImageFormat.YUV_420_888, 2).apply {
            setOnImageAvailableListener({ r ->
                val image = r.acquireLatestImage() ?: return@setOnImageAvailableListener
                try {
                    val y = extractY(image)
                    val bmp = grayscaleBitmap(y, width, height)
                    listener.onFrame(y, bmp, width, height, image.timestamp)
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

    private fun extractY(image: android.media.Image): ByteArray {
        val plane = image.planes[0]
        val buf = plane.buffer
        val rowStride = plane.rowStride
        val out = ByteArray(width * height)
        if (rowStride == width) {
            buf.get(out, 0, width * height)
        } else {
            for (row in 0 until height) {
                buf.position(row * rowStride)
                buf.get(out, row * width, width)
            }
        }
        return out
    }

    private fun grayscaleBitmap(y: ByteArray, w: Int, h: Int): Bitmap {
        val px = IntArray(w * h)
        for (i in 0 until w * h) {
            val v = y[i].toInt() and 0xFF
            px[i] = (0xFF shl 24) or (v shl 16) or (v shl 8) or v
        }
        return Bitmap.createBitmap(px, w, h, Bitmap.Config.ARGB_8888)
    }
}
