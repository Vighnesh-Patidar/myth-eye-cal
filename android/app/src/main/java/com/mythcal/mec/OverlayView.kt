package com.mythcal.mec

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View

/**
 * On-device pose telemetry: draws the live MediaPipe landmarks as the 17-keypoint
 * skeleton (the same slots fusion uses, via MP33TO17 + BONES) directly over the
 * camera preview, coloured by per-point visibility. This is what the device
 * actually recognises — green/steady means good tracking, red/jittery means poor.
 *
 * Landmarks arrive normalised [0,1] in the upright frame fed to MediaPipe and are
 * stretched to the view bounds; on most phones in portrait that overlays the
 * preview closely. Set [mirror] for a front-facing (selfie) camera.
 */
class OverlayView(context: Context, attrs: AttributeSet? = null) : View(context, attrs) {

    companion object {
        // MediaPipe 33-landmark index per 17-skeleton slot (matches
        // kMediapipe33To17, with the right ankle (28) kept for slot 16).
        private val SLOTS = intArrayOf(0, 2, 5, 7, 8, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28)
        // 17-skeleton edges (matches viewer BONES / §6.2).
        private val BONES = arrayOf(
            intArrayOf(0, 1), intArrayOf(0, 2), intArrayOf(1, 3), intArrayOf(2, 4), // face
            intArrayOf(5, 6),                                                        // shoulders
            intArrayOf(5, 7), intArrayOf(7, 9), intArrayOf(6, 8), intArrayOf(8, 10),// arms
            intArrayOf(5, 11), intArrayOf(6, 12), intArrayOf(11, 12),               // torso + hips
            intArrayOf(11, 13), intArrayOf(13, 15), intArrayOf(12, 14), intArrayOf(14, 16) // legs
        )
        private const val N_KP = 17
        private const val VIS_MIN = 0.3f
    }

    var mirror: Boolean = false

    // Rotation (degrees, CW) applied to the landmarks before they are drawn, to
    // undo the camera pipeline's upright rotation so the skeleton lines up with
    // the (landscape) preview. Set by MainActivity from the camera.
    var rotationDeg: Int = 0

    // Flattened 17-skeleton: [x,y,vis] per slot, in normalised coords.
    private val pts = FloatArray(N_KP * 3)
    private var hasPose = false

    private val bonePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE; strokeWidth = 8f; strokeCap = Paint.Cap.ROUND
    }
    private val jointPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }

    /** landmarks = MediaPipe float[count*4] (x,y,z,visibility). count <= 33. */
    fun setLandmarks(landmarks: FloatArray, count: Int) {
        if (count <= 0) { hasPose = false; postInvalidate(); return }
        for (i in 0 until N_KP) {
            val mp = SLOTS[i]
            val base = mp * 4
            if (base + 3 < landmarks.size && mp < count) {
                pts[i * 3] = landmarks[base]
                pts[i * 3 + 1] = landmarks[base + 1]
                pts[i * 3 + 2] = landmarks[base + 3]
            } else {
                pts[i * 3 + 2] = 0f // visibility 0 = skip
            }
        }
        hasPose = true
        postInvalidate()
    }

    private fun colourFor(vis: Float): Int {
        val v = vis.coerceIn(0f, 1f)
        // red (low) -> amber -> green (high)
        val r = ((1f - v) * 255).toInt()
        val g = (v * 220).toInt()
        return Color.argb(230, r, g, 80)
    }

    // Rotate a normalised point (CW) then map to view pixels, applying mirror.
    private fun mapX(nx: Float, ny: Float): Float {
        val rx = when (((rotationDeg % 360) + 360) % 360) {
            90 -> 1f - ny
            180 -> 1f - nx
            270 -> ny
            else -> nx
        }
        return (if (mirror) 1f - rx else rx) * width
    }
    private fun mapY(nx: Float, ny: Float): Float {
        val ry = when (((rotationDeg % 360) + 360) % 360) {
            90 -> nx
            180 -> 1f - ny
            270 -> 1f - nx
            else -> ny
        }
        return ry * height
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (!hasPose) return
        for (b in BONES) {
            val a = b[0]; val c = b[1]
            val va = pts[a * 3 + 2]; val vc = pts[c * 3 + 2]
            if (va < VIS_MIN || vc < VIS_MIN) continue
            bonePaint.color = colourFor(minOf(va, vc))
            canvas.drawLine(mapX(pts[a * 3], pts[a * 3 + 1]), mapY(pts[a * 3], pts[a * 3 + 1]),
                            mapX(pts[c * 3], pts[c * 3 + 1]), mapY(pts[c * 3], pts[c * 3 + 1]), bonePaint)
        }
        for (i in 0 until N_KP) {
            val v = pts[i * 3 + 2]
            if (v < VIS_MIN) continue
            jointPaint.color = colourFor(v)
            canvas.drawCircle(mapX(pts[i * 3], pts[i * 3 + 1]), mapY(pts[i * 3], pts[i * 3 + 1]), 10f, jointPaint)
        }
    }
}
