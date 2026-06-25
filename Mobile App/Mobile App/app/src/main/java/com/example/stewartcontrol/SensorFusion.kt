package com.example.stewartcontrol

import kotlin.math.*

/**
 * Madgwick AHRS filter — fuses gyroscope + accelerometer into stable
 * roll / pitch / yaw angles without drift.
 *
 * Based on: Madgwick, S.O.H. (2010). "An efficient orientation filter for
 * inertial and inertial/magnetic sensor arrays."
 *
 * Usage:
 *   val fusion = SensorFusion(beta = 0.1f)
 *   // call in sensor loop:
 *   fusion.update(gx, gy, gz, ax, ay, az, dt)
 *   val (roll, pitch, yaw) = fusion.getEulerDeg()
 */
class SensorFusion(private var beta: Float = 0.1f) {

    // Quaternion state [w, x, y, z]
    private var q0 = 1f; private var q1 = 0f
    private var q2 = 0f; private var q3 = 0f

    /**
     * Update the filter.
     * @param gx/gy/gz  gyroscope in rad/s
     * @param ax/ay/az  accelerometer (any unit, will be normalised)
     * @param dt        time delta in seconds
     */
    fun update(
        gx: Float, gy: Float, gz: Float,
        ax: Float, ay: Float, az: Float,
        dt: Float,
    ) {
        var normA = sqrt((ax * ax) + (ay * ay) + (az * az))
        if (normA == 0f) return
        normA = 1f / normA
        val axN = ax * normA; val ayN = ay * normA; val azN = az * normA

        // Estimated gravity direction from quaternion
        val vx = 2f * (q1 * q3 - q0 * q2)
        val vy = 2f * (q0 * q1 + q2 * q3)
        val vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        // Error (cross product of estimated vs measured)
        val ex = ayN * vz - azN * vy
        val ey = azN * vx - axN * vz
        val ez = axN * vy - ayN * vx

        // Apply feedback to gyro rates
        val gxF = gx + 2f * beta * ex
        val gyF = gy + 2f * beta * ey
        val gzF = gz + 2f * beta * ez

        // Integrate quaternion
        val half = 0.5f * dt
        q0 += (-q1 * gxF - q2 * gyF - q3 * gzF) * half
        q1 += ( q0 * gxF + q2 * gzF - q3 * gyF) * half
        q2 += ( q0 * gyF - q1 * gzF + q3 * gxF) * half
        q3 += ( q0 * gzF + q1 * gyF - q2 * gxF) * half

        // Normalise quaternion
        var normQ = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
        if (normQ == 0f) normQ = 1f
        normQ = 1f / normQ
        q0 *= normQ; q1 *= normQ; q2 *= normQ; q3 *= normQ
    }

    /** Returns (roll, pitch, yaw) in degrees. Yaw is relative (no magnetometer). */
    fun getEulerDeg(): Triple<Float, Float, Float> {
        val roll  = atan2(2f*(q0*q1 + q2*q3), 1f - 2f*(q1*q1 + q2*q2))
        val pitch = asin( 2f*(q0*q2 - q3*q1))
        val yaw   = atan2(2f*(q0*q3 + q1*q2), 1f - 2f*(q2*q2 + q3*q3))
        return Triple(
            Math.toDegrees(roll.toDouble()).toFloat(),
            Math.toDegrees(pitch.toDouble()).toFloat(),
            Math.toDegrees(yaw.toDouble()).toFloat()
        )
    }

    /** Reset filter to upright position (call on "Calibrate" button). */
    fun reset() { q0 = 1f; q1 = 0f; q2 = 0f; q3 = 0f }

    fun setBeta(b: Float) { beta = b }
}
