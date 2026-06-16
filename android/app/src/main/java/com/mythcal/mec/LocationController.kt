package com.mythcal.mec

import android.content.Context
import android.location.Location
import android.location.LocationListener
import android.location.LocationManager

/**
 * GPS co-localization (§15.12). The origin is an ABSOLUTE WGS84 coordinate
 * (lat/lon/alt) set explicitly — ideally a surveyed/benchmark point — so the
 * swarm shares one known frame and ground-truth correction is a fixed offset.
 * Each phone's position is the ENU metre offset of its own fix from that origin.
 *
 * Listens on GPS *and* NETWORK providers: GPS is precise but useless indoors,
 * NETWORK (Wi-Fi/cell) gives a coarse fix indoors so the UI always reflects
 * state. CAVEAT: consumer GPS is ~3-10 m; for indoor accuracy use surveyed pins.
 */
class LocationController(context: Context) : LocationListener {
    private val lm = context.getSystemService(Context.LOCATION_SERVICE) as LocationManager

    @Volatile var lat = 0.0; @Volatile var lon = 0.0; @Volatile var alt = 0.0
    @Volatile var accuracyM = -1f; @Volatile var hasFix = false
    @Volatile var provider = ""

    private var oLat = 0.0; private var oLon = 0.0; private var oAlt = 0.0
    var hasOrigin = false; private set

    fun start() {
        val providers = listOf(
            LocationManager.GPS_PROVIDER,
            LocationManager.NETWORK_PROVIDER,
            LocationManager.PASSIVE_PROVIDER)
        for (p in providers) {
            try {
                lm.getLastKnownLocation(p)?.let { update(it) }
                if (lm.isProviderEnabled(p)) lm.requestLocationUpdates(p, 500L, 0f, this)
            } catch (_: SecurityException) { /* permission missing */
            } catch (_: IllegalArgumentException) { /* provider absent */ }
        }
    }

    fun stop() { try { lm.removeUpdates(this) } catch (_: SecurityException) {} }

    override fun onLocationChanged(loc: Location) = update(loc)
    override fun onProviderEnabled(provider: String) {}
    override fun onProviderDisabled(provider: String) {}

    private fun update(loc: Location) {
        lat = loc.latitude; lon = loc.longitude; alt = loc.altitude
        accuracyM = loc.accuracy; provider = loc.provider ?: ""; hasFix = true
    }

    fun servicesEnabled(): Boolean = try {
        lm.isProviderEnabled(LocationManager.GPS_PROVIDER) ||
            lm.isProviderEnabled(LocationManager.NETWORK_PROVIDER)
    } catch (_: Exception) { false }

    fun setOrigin(latDeg: Double, lonDeg: Double, altM: Double) {
        oLat = latDeg; oLon = lonDeg; oAlt = altM; hasOrigin = true
    }
    fun setOriginHere(): Boolean { if (!hasFix) return false; setOrigin(lat, lon, alt); return true }
    fun origin(): DoubleArray = doubleArrayOf(oLat, oLon, oAlt)

    /** ENU (east, north, up) metres of the current fix from the absolute origin. */
    fun enu(): FloatArray? {
        if (!hasFix || !hasOrigin) return null
        val d = Math.PI / 180.0
        val R = 6378137.0
        val north = (lat - oLat) * d * R
        val east = (lon - oLon) * d * R * Math.cos(oLat * d)
        val up = alt - oAlt
        return floatArrayOf(east.toFloat(), north.toFloat(), up.toFloat())
    }
}
