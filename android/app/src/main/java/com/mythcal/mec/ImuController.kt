package com.mythcal.mec

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager

/**
 * Feeds accelerometer + gyroscope into the native IMUIntegrator at the sensor's
 * fastest rate (~200Hz on most devices). Accelerometer (TYPE_ACCELEROMETER)
 * includes gravity, which is exactly the specific force the integrator expects.
 */
class ImuController(context: Context, private val handle: Long) : SensorEventListener {
    private val sm = context.getSystemService(Context.SENSOR_SERVICE) as SensorManager
    private val accel = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    private val gyro = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
    private val lastA = FloatArray(3)
    private var lastGyroNs = 0L

    fun start() {
        sm.registerListener(this, accel, SensorManager.SENSOR_DELAY_FASTEST)
        sm.registerListener(this, gyro, SensorManager.SENSOR_DELAY_FASTEST)
    }

    fun stop() = sm.unregisterListener(this)

    override fun onSensorChanged(e: SensorEvent) {
        when (e.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                lastA[0] = e.values[0]; lastA[1] = e.values[1]; lastA[2] = e.values[2]
            }
            Sensor.TYPE_GYROSCOPE -> {
                val dt = if (lastGyroNs == 0L) 0.005f else (e.timestamp - lastGyroNs) * 1e-9f
                lastGyroNs = e.timestamp
                MecNative.nativeOnImuSample(
                    handle, lastA[0], lastA[1], lastA[2],
                    e.values[0], e.values[1], e.values[2], dt
                )
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {}
}
