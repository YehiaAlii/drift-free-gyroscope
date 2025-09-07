package com.example.kalmanfilter

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

class ExtendedKalmanFilter {

    // Baseline process noise values for dynamic tuning
    private val baseProcessNoiseAngles = 0.01f
    private val baseProcessNoiseBias = 0.0001f

    // Baseline measurement noise values
    private val baseMeasurementNoiseRoll = 0.1f
    private val baseMeasurementNoisePitch = 0.1f

    // Dynamic tuning parameters
    private val maxDynamicFactor = 10.0f  // Maximum scaling for process noise
    private val minAccelTrust = 0.5f      // Minimum scaling for measurement noise
    private val maxAccelTrust = 5.0f      // Maximum scaling for measurement noise

    // State vector: [roll, pitch, yaw, bias_gyro_x, bias_gyro_y, bias_gyro_z]
    private val state = FloatArray(6) { 0f }

    // State covariance matrix (6x6)
    private val P = Array(6) { FloatArray(6) { 0f } }

    // Process noise covariance (how much we expect the state to change)
    private val Q = Array(6) { FloatArray(6) { 0f } }

    // Measurement noise covariance (how noisy we expect the accelerometer to be)
    private val R = Array(2) { FloatArray(2) { 0f } }

    // Earth's gravity constant
    private val GRAVITY = 9.81f

    init {
        // Initial state covariance (uncertainty in our initial state)
        for (i in 0 until 6) {
            P[i][i] = if (i < 3) 1.0f else 0.1f
        }

        // Process noise (uncertainty in our model)
        for (i in 0 until 6) {
            Q[i][i] = if (i < 3) baseProcessNoiseAngles else baseProcessNoiseBias
        }

        // Measurement noise (uncertainty in accelerometer readings)
        R[0][0] = baseMeasurementNoiseRoll
        R[1][1] = baseMeasurementNoisePitch
    }

    // Get the current state values
    fun getRoll() = state[0]
    fun getPitch() = state[1]
    fun getYaw() = state[2]
    fun getBiasX() = state[3]
    fun getBiasY() = state[4]
    fun getBiasZ() = state[5]

    // Reset the filter
    fun reset() {
        for (i in 0 until 3) {  // Only reset orientation, not biases
            state[i] = 0f
        }

        // Reset covariance for orientation
        for (i in 0 until 3) {
            for (j in 0 until 6) {
                if (i == j) {
                    P[i][j] = 1.0f  // Reset diagonal elements
                } else {
                    P[i][j] = 0f
                    P[j][i] = 0f
                }
            }
        }
    }

    // Prediction step using gyroscope data
    fun predict(gyroX: Float, gyroY: Float, gyroZ: Float, dt: Float) {
        // Calculate gyro magnitude for dynamic tuning
        val gyroMagnitude = sqrt(gyroX*gyroX + gyroY*gyroY + gyroZ*gyroZ)

        // Adjust process noise based on motion intensity
        adjustProcessNoise(gyroMagnitude)

        // Current state values
        val roll = state[0]
        val pitch = state[1]
        val yaw = state[2]
        val biasX = state[3]
        val biasY = state[4]
        val biasZ = state[5]

        // Apply bias correction to gyroscope readings
        val correctedGyroX = gyroX - biasX
        val correctedGyroY = gyroY - biasY
        val correctedGyroZ = gyroZ - biasZ

        // Calculate Euler angle derivatives based on gyroscope
        val rollRate = correctedGyroX +
                sin(roll) * tan(pitch) * correctedGyroY +
                cos(roll) * tan(pitch) * correctedGyroZ

        val pitchRate = cos(roll) * correctedGyroY -
                sin(roll) * correctedGyroZ

        val yawRate = (sin(roll) / cos(pitch)) * correctedGyroY +
                (cos(roll) / cos(pitch)) * correctedGyroZ

        // Integrate to get new state
        state[0] += rollRate * dt
        state[1] += pitchRate * dt
        state[2] += yawRate * dt
        // Bias states remain unchanged in prediction

        // Calculate state transition Jacobian
        val F = calculateStateTransitionJacobian(correctedGyroX, correctedGyroY, correctedGyroZ, dt)

        // Create process noise matrix for this time step
        val Qt = Array(6) { i -> FloatArray(6) { j -> if (i == j) Q[i][i] * dt else 0f } }

        // Update state covariance: P = F*P*F^T + Q
        val Ft = matrixTranspose(F)
        val FP = matrixMultiply(F, P)
        val FPFt = matrixMultiply(FP, Ft)
        val result = matrixAdd(FPFt, Qt)
        for (i in 0 until 6) {
            for (j in 0 until 6) {
                P[i][j] = result[i][j]
            }
        }
        handleSingularity()
    }

    // Update step using accelerometer data
    fun update(accelX: Float, accelY: Float, accelZ: Float) {
        // Get current state values
        val roll = state[0]
        val pitch = state[1]

        // Calculate expected accelerometer readings based on current orientation
        val expectedAccelX = -GRAVITY * sin(pitch)
        val expectedAccelY = GRAVITY * sin(roll) * cos(pitch)
        val expectedAccelZ = GRAVITY * cos(roll) * cos(pitch)

        // Calculate magnitude of acceleration
        val accelMagnitude = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ)

        // Adjust measurement noise based on accelerometer trust
        adjustMeasurementNoise(accelMagnitude)

        // Only trust accelerometer when it's measuring close to gravity
        if (abs(accelMagnitude - GRAVITY) > 1.0f) {
            return  // Skip update if there's significant linear acceleration
        }

        // Calculate roll and pitch from accelerometer
        val measuredRoll = atan2(accelY, accelZ)
        val measuredPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ))

        // 1. Create measurement vector (z)
        val z = floatArrayOf(measuredRoll, measuredPitch)

        // 2. Create predicted measurement vector (h)
        val h = floatArrayOf(roll, pitch)

        // 3. Calculate innovation/residual (y = z - h)
        val y = floatArrayOf(z[0] - h[0], z[1] - h[1])

        // 4. Get measurement Jacobian (H)
        val H = calculateMeasurementJacobian()

        // 5. Calculate innovation covariance (S = H*P*H^T + R)
        val Ht = matrixTranspose(H)
        val HP = matrixMultiply(H, P)
        val HPHt = matrixMultiply(HP, Ht)

        // Create R matrix from diagonal elements
        val RMatrix = Array(2) { i -> FloatArray(2) { j -> if (i == j) R[i][i] else 0f } }

        val S = matrixAdd(HPHt, RMatrix)

        // 6. Calculate Kalman gain (K = P*H^T*S^-1)
        val PHt = matrixMultiply(P, Ht)
        val Sinv = inverse2x2(S)
        val K = matrixMultiply(PHt, Sinv)

        // 7. Update state (x = x + K*y)
        for (i in 0 until 6) {
            var stateUpdate = 0f
            for (j in 0 until 2) {
                stateUpdate += K[i][j] * y[j]
            }
            state[i] += stateUpdate
        }

        // 8. Update state covariance (P = (I - K*H)*P)
        // Create identity matrix
        val I = Array(6) { i -> FloatArray(6) { j -> if (i == j) 1f else 0f } }

        // Calculate K*H
        val KH = matrixMultiply(K, H)

        // Calculate I - K*H
        val IKH = Array(6) { i -> FloatArray(6) { j -> I[i][j] - KH[i][j] } }

        // Calculate (I - K*H)*P
        val IKHP = matrixMultiply(IKH, P)

        // Update P
        for (i in 0 until 6) {
            for (j in 0 until 6) {
                P[i][j] = IKHP[i][j]
            }
        }

        handleSingularity()
    }

    private fun matrixMultiply(A: Array<FloatArray>, B: Array<FloatArray>): Array<FloatArray> {
        val m = A.size
        val n = B.size
        val p = B[0].size

        val C = Array(m) { FloatArray(p) { 0f } }

        for (i in 0 until m) {
            for (j in 0 until p) {
                for (k in 0 until n) {
                    C[i][j] += A[i][k] * B[k][j]
                }
            }
        }

        return C
    }

    private fun matrixTranspose(A: Array<FloatArray>): Array<FloatArray> {
        val rows = A.size
        val cols = A[0].size

        val result = Array(cols) { FloatArray(rows) { 0f } }

        for (i in 0 until rows) {
            for (j in 0 until cols) {
                result[j][i] = A[i][j]
            }
        }

        return result
    }

    private fun matrixAdd(A: Array<FloatArray>, B: Array<FloatArray>): Array<FloatArray> {
        val rows = A.size
        val cols = A[0].size

        val result = Array(rows) { FloatArray(cols) { 0f } }

        for (i in 0 until rows) {
            for (j in 0 until cols) {
                result[i][j] = A[i][j] + B[i][j]
            }
        }

        return result
    }

    private fun matrixSubtract(A: Array<FloatArray>, B: Array<FloatArray>): Array<FloatArray> {
        val rows = A.size
        val cols = A[0].size

        val result = Array(rows) { FloatArray(cols) { 0f } }

        for (i in 0 until rows) {
            for (j in 0 until cols) {
                result[i][j] = A[i][j] - B[i][j]
            }
        }

        return result
    }

    private fun inverse2x2(A: Array<FloatArray>): Array<FloatArray> {
        val det = A[0][0] * A[1][1] - A[0][1] * A[1][0]

        if (abs(det) < 1e-6f) {
            // Matrix is singular, return identity
            return Array(2) { i -> FloatArray(2) { j -> if (i == j) 1f else 0f } }
        }

        val invDet = 1f / det

        val result = Array(2) { FloatArray(2) }
        result[0][0] = A[1][1] * invDet
        result[0][1] = -A[0][1] * invDet
        result[1][0] = -A[1][0] * invDet
        result[1][1] = A[0][0] * invDet

        return result
    }

    private fun adjustProcessNoise(gyroMagnitude: Float) {
        // Calculate dynamic factor based on gyro magnitude
        // Higher gyro values mean more dynamic motion, so we increase process noise
        val dynamicFactor = 1.0f + (gyroMagnitude * 2.0f).coerceAtMost(maxDynamicFactor - 1.0f)

        // Scale process noise for orientation states
        for (i in 0 until 3) {
            Q[i][i] = baseProcessNoiseAngles * dynamicFactor
        }

        // Bias states have less dynamic adjustment
        for (i in 3 until 6) {
            Q[i][i] = baseProcessNoiseBias * (1.0f + (dynamicFactor - 1.0f) * 0.1f)
        }
    }

    private fun adjustMeasurementNoise(accelMagnitude: Float, gravityMagnitude: Float = GRAVITY) {
        // Calculate how far acceleration magnitude is from expected gravity
        val accelError = abs(accelMagnitude - gravityMagnitude)

        // Calculate trust factor: 1.0 means perfect trust, higher values mean less trust
        // When accelError is 0, trustFactor should be 1.0 (we fully trust the measurement)
        // As accelError increases, we trust it less
        val trustFactor = (1.0f + (accelError * 5.0f)).coerceIn(minAccelTrust, maxAccelTrust)

        // Update measurement noise covariance
        R[0][0] = baseMeasurementNoiseRoll * trustFactor
        R[1][1] = baseMeasurementNoisePitch * trustFactor
    }

    /**
     * Calculate the state transition Jacobian (F)
     * This matrix represents how the state derivatives depend on the state
     */
    private fun calculateStateTransitionJacobian(
        gyroX: Float, gyroY: Float, gyroZ: Float,
        dt: Float
    ): Array<FloatArray> {
        val roll = state[0]
        val pitch = state[1]

        // Initialize Jacobian with identity matrix
        val F = Array(6) { i -> FloatArray(6) { j -> if (i == j) 1f else 0f } }

        // Non-zero partial derivatives for roll rate
        // d(roll_rate)/d(roll)
        F[0][0] += (cos(roll) * tan(pitch) * gyroY - sin(roll) * tan(pitch) * gyroZ) * dt

        // d(roll_rate)/d(pitch)
        val secPitch = 1f / cos(pitch)
        F[0][1] += (sin(roll) * secPitch * secPitch * gyroY +
                cos(roll) * secPitch * secPitch * gyroZ) * dt

        // d(roll_rate)/d(bias_x)
        F[0][3] += -dt

        // d(roll_rate)/d(bias_y)
        F[0][4] += -sin(roll) * tan(pitch) * dt

        // d(roll_rate)/d(bias_z)
        F[0][5] += -cos(roll) * tan(pitch) * dt

        // Non-zero partial derivatives for pitch rate
        // d(pitch_rate)/d(roll)
        F[1][0] += (-sin(roll) * gyroY - cos(roll) * gyroZ) * dt

        // d(pitch_rate)/d(bias_y)
        F[1][4] += -cos(roll) * dt

        // d(pitch_rate)/d(bias_z)
        F[1][5] += sin(roll) * dt

        // Non-zero partial derivatives for yaw rate
        // d(yaw_rate)/d(roll)
        F[2][0] += ((cos(roll) / cos(pitch)) * gyroY -
                (sin(roll) / cos(pitch)) * gyroZ) * dt

        // d(yaw_rate)/d(pitch)
        F[2][1] += ((sin(roll) * sin(pitch) / (cos(pitch) * cos(pitch))) * gyroY +
                (cos(roll) * sin(pitch) / (cos(pitch) * cos(pitch))) * gyroZ) * dt

        // d(yaw_rate)/d(bias_y)
        F[2][4] += -(sin(roll) / cos(pitch)) * dt

        // d(yaw_rate)/d(bias_z)
        F[2][5] += -(cos(roll) / cos(pitch)) * dt

        return F
    }

    /**
     * Calculate the measurement Jacobian (H)
     * This matrix represents how the expected measurements depend on the state
     */
    private fun calculateMeasurementJacobian(): Array<FloatArray> {
        val roll = state[0]
        val pitch = state[1]

        // Initialize measurement Jacobian (2x6 matrix)
        val H = Array(2) { FloatArray(6) { 0f } }

        // Partial derivatives for expected roll measurement
        // d(expected_roll)/d(roll)
        H[0][0] = 1f  // Roll measurement directly corresponds to roll state

        // Partial derivatives for expected pitch measurement
        // d(expected_pitch)/d(pitch)
        H[1][1] = 1f  // Pitch measurement directly corresponds to pitch state

        return H
    }

    private fun handleSingularity() {
        val pitch = state[1]

        // Check if pitch is near ±90 degrees (π/2 radians)
        val nearSingularity = abs(abs(pitch) - PI.toFloat()/2) < 0.1f

        if (nearSingularity) {
            // Limit pitch to slightly less than ±90° to avoid singularity
            val safetyMargin = 0.01f
            val maxPitch = PI.toFloat()/2 - safetyMargin

            // Clamp pitch value
            state[1] = state[1].coerceIn(-maxPitch, maxPitch)

            // Increase uncertainty in roll and yaw when near singularity
            // as they become more correlated and less reliable
            P[0][0] = maxOf(P[0][0], 0.5f)  // Roll uncertainty
            P[2][2] = maxOf(P[2][2], 0.5f)  // Yaw uncertainty
        }
    }

    fun setYaw(yaw: Float) {
        state[2] = yaw
    }

    fun updateYaw(yaw: Float) {
        // Only update yaw if it's not too different from current estimate
        // to avoid sudden jumps
        val currentYaw = state[2]
        var yawDiff = yaw - currentYaw

        // Normalize the difference to -π to π
        while (yawDiff > PI) yawDiff -= 2*PI.toFloat()
        while (yawDiff < -PI) yawDiff += 2*PI.toFloat()

        // If the difference is small, do a smooth update
        // If it's large, it might be a flip or magnetometer error
        if (abs(yawDiff) < PI/4) {
            // Use a small weight factor to smoothly update yaw
            state[2] = currentYaw + 0.05f * yawDiff

            // Reduce yaw uncertainty since we have an absolute reference
            P[2][2] = maxOf(0.1f, P[2][2] * 0.95f)
        }
    }

    fun updateWithYawMeasurement(measuredYaw: Float) {
        val yaw = state[2]

        // Calculate difference (innovation) with proper angle wrapping
        var innovation = measuredYaw - yaw
        while (innovation > PI) innovation -= 2*PI.toFloat()
        while (innovation < -PI) innovation += 2*PI.toFloat()

        // Create measurement matrix (1x6 with 1 at yaw position)
        val H = Array(1) { FloatArray(6) { 0f } }
        H[0][2] = 1f  // Only measuring yaw

        // Create measurement noise - higher than accel because magnetometer
        // can be affected by environmental factors
        val R = Array(1) { FloatArray(1) { 0.3f } }

        // Calculate Kalman gain
        val Ht = matrixTranspose(H)
        val PHt = matrixMultiply(P, Ht)
        val HPHt = matrixMultiply(H, PHt)
        val S = Array(1) { FloatArray(1) { HPHt[0][0] + R[0][0] } }
        val Sinv = Array(1) { FloatArray(1) { 1f / S[0][0] } }
        val K = matrixMultiply(PHt, Sinv)

        // Update state
        for (i in 0 until 6) {
            state[i] += K[i][0] * innovation
        }

        // Update covariance
        val KH = matrixMultiply(K, H)
        val I = Array(6) { i -> FloatArray(6) { j -> if (i == j) 1f else 0f } }
        val IKH = Array(6) { i -> FloatArray(6) { j -> I[i][j] - KH[i][j] } }
        val newP = matrixMultiply(IKH, P)

        for (i in 0 until 6) {
            for (j in 0 until 6) {
                P[i][j] = newP[i][j]
            }
        }
    }
}