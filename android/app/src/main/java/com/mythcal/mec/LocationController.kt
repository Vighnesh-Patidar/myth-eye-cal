package com.mythcal.mec

import android.content.Context
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager

/**
 * Optional GPS co-localization (§15.12). Converts the device's precise location
 * to local ENU metres relative to a shared reference origin: every phone taps
 * "GPS origin" at the same physical spot, then "GPS fill" at its position.
 *
 * CAVEAT: consumer GPS is ~3-10 m and poor/absent indoors — manual pins are
 * better for room-scale through-wall demos; GPS suits outdoor / large layouts.
 */
class LocationController(context: Context) : LocationListener {
    private val lm = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager
    @Volatile private var last: Location? = null
    private var ref: Location? = null

    fun start() {
        try {
            lm.getLastKnownLocation(LocationManager.GPS_PROVIDER)?.let { last = it }
            lm.requestLocationUpdates(LocationManager.GPS_PROVIDER, 500L, 0f, this)
        } catch (_: SecurityException) { /* permission not granted yet */ }
    }

    fun stop() {
        try { lm.removeUpdates(this) } catch (_: SecurityException) {}
    }

    override fun onLocationChanged(loc: Location) { last = loc }

    /** Capture the current fix as the shared origin. Returns false if no fix. */
    fun setOrigin(): Boolean { ref = last; return ref != null }

    /** ENU offset (east, north, up) metres from the origin, or null if unavailable. */
    fun enu(): FloatArray? {
        val l = last ?: return null
        val r = ref ?: return null
        val d = Math.PI / 180.0
        val R = 6378137.0 // WGS84 equatorial radius
        val north = (l.latitude - r.latitude) * d * R
        val east = (l.longitude - r.longitude) * d * R * Math.cos(r.latitude * d)
        val up = l.altitude - r.altitude
        return floatArrayOf(east.toFloat(), north.toFloat(), up.toFloat())
    }

    fun accuracyM(): Float = last?.accuracy ?: -1f
    fun hasFix(): Boolean = last != null
    fun hasOrigin(): Boolean = ref != null
}
