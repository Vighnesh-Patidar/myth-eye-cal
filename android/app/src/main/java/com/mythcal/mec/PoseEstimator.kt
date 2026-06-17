package com.mythcal.mec

import android.content.Context
import android.graphics.Bitmap
import android.util.Log
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.poselandmarker.PoseLandmarker
import kotlin.math.PI
import kotlin.math.abs

/**
 * One-Euro filter (Casiez et al.): adaptive low-pass that kills jitter when the
 * point is still but follows fast motion without lag. Used to steady the noisy
 * per-frame pose landmarks (esp. partial / low-confidence joints).
 */
private class OneEuro(
    private val minCutoff: Float = 1.0f,
    private val beta: Float = 0.02f,
    private val dCutoff: Float = 1.0f,
) {
    private var xPrev = Float.NaN
    private var dxPrev = 0f
    private var tPrevMs = 0L

    private fun alpha(cutoff: Float, dt: Float): Float {
        val tau = 1f / (2f * PI.toFloat() * cutoff)
        return 1f / (1f + tau / dt)
    }

    fun filter(x: Float, tMs: Long): Float {
        if (xPrev.isNaN()) { xPrev = x; tPrevMs = tMs; return x }
        var dt = (tMs - tPrevMs) / 1000f
        if (dt <= 0f) dt = 1f / 30f
        tPrevMs = tMs
        val dx = (x - xPrev) / dt
        val aD = alpha(dCutoff, dt)
        val dxHat = aD * dx + (1f - aD) * dxPrev
        dxPrev = dxHat
        val cutoff = minCutoff + beta * abs(dxHat)
        val a = alpha(cutoff, dt)
        val xHat = a * x + (1f - a) * xPrev
        xPrev = xHat
        return xHat
    }

    fun reset() { xPrev = Float.NaN; dxPrev = 0f }
}

/**
 * MediaPipe Pose Landmarker wrapper (§4.2). Returns 33 body landmarks as
 * float[33*4] = (x_norm, y_norm, z, visibility).
 *
 * Tuned for steady BODY-pose tracking on a single phone (§15.11):
 *  - RunningMode.VIDEO: the detector seeds once, then the tracker follows the
 *    body across frames using the previous ROI. This is far steadier than
 *    per-frame IMAGE detection and removes the "needs the face every frame"
 *    failure of BlazePose's person detector.
 *  - Low min-confidences so partial / side / back-facing bodies keep tracking
 *    instead of dropping out.
 *  - Model preference heavy -> full -> lite: the first asset present in
 *    app/src/main/assets/ is used. Heavy/full recognise the body much better
 *    (worth the extra latency here). At least one pose_landmarker_*.task must
 *    be bundled (see android/README.md).
 *
 * The caller must feed an UPRIGHT RGB bitmap (CameraController already rotates
 * + colour-converts), and strictly increasing timestamps (handled internally).
 */
class PoseEstimator(
    context: Context,
    preferredModels: List<String> = ALL,
) {
    private val landmarker: PoseLandmarker
    val modelName: String
    private var lastTsMs = 0L

    companion object {
        // Heaviest first: more capacity = better recognition, lower frame rate.
        val ALL = listOf(
            "pose_landmarker_heavy.task",
            "pose_landmarker_full.task",
            "pose_landmarker_lite.task",
        )

        // The subset of ALL actually bundled in assets/ (so the UI only offers
        // models that exist and won't throw on selection).
        fun availableModels(context: Context): List<String> {
            val present = try { context.assets.list("")?.toSet() ?: emptySet() }
                          catch (e: Exception) { emptySet() }
            return ALL.filter { it in present }
        }

        // Short display name for a model asset filename.
        fun label(file: String): String = when {
            file.contains("heavy") -> "Heavy"
            file.contains("full")  -> "Full"
            file.contains("lite")  -> "Lite"
            else -> file
        }
    }

    // Per-landmark One-Euro filters (x, y, z) — steadies the 33 BlazePose points.
    private val fx = Array(33) { OneEuro() }
    private val fy = Array(33) { OneEuro() }
    private val fz = Array(33) { OneEuro() }
    private var hadPose = false

    init {
        var built: PoseLandmarker? = null
        var chosen: String? = null
        var lastErr: Exception? = null
        for (model in preferredModels) {
            try {
                val base = BaseOptions.builder().setModelAssetPath(model).build()
                val options = PoseLandmarker.PoseLandmarkerOptions.builder()
                    .setBaseOptions(base)
                    .setRunningMode(RunningMode.VIDEO)
                    .setNumPoses(1)
                    // Steadier hold: detect readily, and keep tracking through
                    // partial occlusion / turning away.
                    .setMinPoseDetectionConfidence(0.3f)
                    .setMinPosePresenceConfidence(0.5f) // drop low-confidence phantom joints
                    .setMinTrackingConfidence(0.3f)
                    .build()
                built = PoseLandmarker.createFromOptions(context, options)
                chosen = model
                break
            } catch (e: Exception) {
                lastErr = e
                Log.w("PoseEstimator", "model '$model' unavailable: ${e.message}")
            }
        }
        landmarker = built ?: throw IllegalStateException(
            "No pose_landmarker_*.task found in assets/ (tried $preferredModels)", lastErr)
        modelName = chosen ?: "?"
        Log.i("PoseEstimator", "pose model: $modelName (VIDEO mode)")
    }

    /**
     * Track the body in [bitmap] (an upright RGB frame). [timestampMs] should be
     * the frame's monotonic time in ms; it is forced strictly increasing for
     * MediaPipe's VIDEO tracker.
     */
    fun detect(bitmap: Bitmap, timestampMs: Long): FloatArray {
        val ts = if (timestampMs <= lastTsMs) lastTsMs + 1 else timestampMs
        lastTsMs = ts
        val result = landmarker.detectForVideo(BitmapImageBuilder(bitmap).build(), ts)
        val poses = result.landmarks()
        if (poses.isEmpty()) {
            // Lost the body: reset filters so re-acquisition doesn't lerp from
            // a stale position.
            if (hadPose) { for (i in 0 until 33) { fx[i].reset(); fy[i].reset(); fz[i].reset() } }
            hadPose = false
            return FloatArray(0)
        }
        hadPose = true
        val lm = poses[0]
        val out = FloatArray(lm.size * 4)
        for (i in lm.indices) {
            // Smooth position; leave visibility raw (used for gating, not display).
            out[i * 4 + 0] = if (i < 33) fx[i].filter(lm[i].x(), ts) else lm[i].x()
            out[i * 4 + 1] = if (i < 33) fy[i].filter(lm[i].y(), ts) else lm[i].y()
            out[i * 4 + 2] = if (i < 33) fz[i].filter(lm[i].z(), ts) else lm[i].z()
            out[i * 4 + 3] = lm[i].visibility().orElse(0f)
        }
        return out
    }

    fun close() = landmarker.close()
}
