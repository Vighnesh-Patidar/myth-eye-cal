package com.mythcal.mec

import android.content.Context
import android.graphics.Bitmap
import com.google.mediapipe.framework.image.BitmapImageBuilder
import com.google.mediapipe.tasks.core.BaseOptions
import com.google.mediapipe.tasks.vision.core.RunningMode
import com.google.mediapipe.tasks.vision.poselandmarker.PoseLandmarker

/**
 * MediaPipe Pose Landmarker wrapper (§4.2). Returns 33 world-convention
 * landmarks as float[33*4] = (x_norm, y_norm, z, visibility).
 *
 * The model asset `pose_landmarker_lite.task` must be placed in
 * app/src/main/assets/ (see android/README.md) — it is NOT bundled in the repo.
 */
class PoseEstimator(context: Context, modelAsset: String = "pose_landmarker_lite.task") {
    private val landmarker: PoseLandmarker

    init {
        val base = BaseOptions.builder().setModelAssetPath(modelAsset).build()
        val options = PoseLandmarker.PoseLandmarkerOptions.builder()
            .setBaseOptions(base)
            .setRunningMode(RunningMode.IMAGE)
            .setNumPoses(1)
            .build()
        landmarker = PoseLandmarker.createFromOptions(context, options)
    }

    fun detect(bitmap: Bitmap): FloatArray {
        val result = landmarker.detect(BitmapImageBuilder(bitmap).build())
        val poses = result.landmarks()
        if (poses.isEmpty()) return FloatArray(0)
        val lm = poses[0]
        val out = FloatArray(lm.size * 4)
        for (i in lm.indices) {
            out[i * 4 + 0] = lm[i].x()
            out[i * 4 + 1] = lm[i].y()
            out[i * 4 + 2] = lm[i].z()
            out[i * 4 + 3] = lm[i].visibility().orElse(0f)
        }
        return out
    }

    fun close() = landmarker.close()
}
