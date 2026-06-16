package com.mythcal.mec

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager

/**
 * Feeds the absolute device->world orientation into native from the fused
 * rotation-vector sensor (accelerometer + gyroscope + magnetometer). Because it
 * is referenced to gravity + magnetic north, every phone shares the same world
 * frame (co-localization, §15.12) and the orientation does not drift.
 */
class OrientationController(context: Context, private val handle: Long) : SensorEventListener {
    private val sm = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val rv = sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    private val q = FloatArray(4)

    fun start() = sm.registerListener(this, rv, SensorManager.SENSOR_DELAY_GAME)
    fun stop() = sm.unregisterListener(this)

    override fun onSensorChanged(e: SensorEvent) {
        if (e.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return
        SensorManager.getQuaternionFromVector(q, e.values) // q = [w, x, y, z]
        MecNative.nativeSetOrientation(handle, q[0], q[1], q[2], q[3])
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
}
