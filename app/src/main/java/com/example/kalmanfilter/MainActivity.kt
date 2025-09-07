package com.example.kalmanfilter
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import androidx.navigation.NavController
import androidx.compose.foundation.layout.*
import kotlin.math.sqrt
import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import androidx.compose.material3.ExperimentalMaterial3Api
import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Text
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.material3.MaterialTheme
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight

class MainActivity : ComponentActivity(), SensorEventListener {

    private val ekf = ExtendedKalmanFilter()

    private lateinit var sensorManager: SensorManager
    private var gyroscope: Sensor? = null

    private var isGyroscopeRunning by mutableStateOf(true)
    private var gyroscopeX by mutableStateOf(0f)
    private var gyroscopeY by mutableStateOf(0f)
    private var gyroscopeZ by mutableStateOf(0f)

    // Theta (angle) values from integration
    private var thetaX by mutableStateOf(0f)
    private var thetaY by mutableStateOf(0f)
    private var thetaZ by mutableStateOf(0f)

    // For integration calculation
    private var lastTimestamp: Long = 0
    private var isFirstReading = true

    // ZUPT related variables
    private val movementThreshold = Constants.MOVEMENT_THRESHOLD  // Adjust based on testing
    private var isStationary by mutableStateOf(false)
    private val staticWindowSize = Constants.STATIC_WINDOW_SIZE  // Number of samples to confirm static state
    private val recentGyroMagnitudes = ArrayDeque<Float>(staticWindowSize)

    // Bias estimation variables
    private var gyroBiasX = 0f
    private var gyroBiasY = 0f
    private var gyroBiasZ = 0f
    private var biasEstimationCount = 0
    private val biasUpdateRate = Constants.BIAS_UPDATE_RATE  // Low-pass filter factor for bias updates

    private val gyroscopeReadings = mutableStateListOf<SensorReading>()
    private val maxDataPoints = Constants.MAX_DATA_POINTS

    // Add these properties to MainActivity
    private var accelerometer: Sensor? = null
    private var accelX by mutableStateOf(0f)
    private var accelY by mutableStateOf(0f)
    private var accelZ by mutableStateOf(0f)

    private var filteredRoll by mutableStateOf(0f)
    private var filteredPitch by mutableStateOf(0f)
    private var filteredYaw by mutableStateOf(0f)

    private var magnetometer: Sensor? = null
    private var magX by mutableStateOf(0f)
    private var magY by mutableStateOf(0f)
    private var magZ by mutableStateOf(0f)

    private var absoluteYaw by mutableStateOf(0f)
    companion object {
        private const val STORAGE_PERMISSION_CODE = Constants.STORAGE_PERMISSION_CODE
    }

    private fun checkAndRequestPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            if (checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
                requestPermissions(
                    arrayOf(Manifest.permission.WRITE_EXTERNAL_STORAGE),
                    STORAGE_PERMISSION_CODE
                )
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        checkAndRequestPermissions()
        sensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        magnetometer = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        enableEdgeToEdge()
        setContent {
            MaterialTheme {
                MainScreen(
                    gyroscopeX = gyroscopeX,
                    gyroscopeY = gyroscopeY,
                    gyroscopeZ = gyroscopeZ,
                    accelX = accelX,
                    accelY = accelY,
                    accelZ = accelZ,
                    thetaX = thetaX,
                    thetaY = thetaY,
                    thetaZ = thetaZ,
                    magX = magX,
                    magY = magY,
                    magZ = magZ,
                    filteredRoll = filteredRoll,
                    filteredPitch = filteredPitch,
                    absoluteYaw = absoluteYaw,
                    isGyroscopeRunning = isGyroscopeRunning,
                    gyroscopeReadings = gyroscopeReadings.toList(),
                    isStationary = isStationary,
                    onGyroscopeToggle = { toggleGyroscope() },
                    onResetTheta = { resetTheta() }
                )
            }
        }
    }

    private fun calculateAbsoluteOrientation(): FloatArray? {
        val rotationMatrix = FloatArray(9)
        val orientationAngles = FloatArray(3)

        // Check if we have valid accelerometer and magnetometer data
        if ((accelX != 0f || accelY != 0f || accelZ != 0f) &&
            (magX != 0f || magY != 0f || magZ != 0f)) {

            try {
                // Get rotation matrix from sensor data
                val success = SensorManager.getRotationMatrix(
                    rotationMatrix,
                    null,
                    floatArrayOf(accelX, accelY, accelZ),
                    floatArrayOf(magX, magY, magZ)
                )

                if (success) {
                    // Get orientation angles from rotation matrix
                    SensorManager.getOrientation(rotationMatrix, orientationAngles)
                    return orientationAngles
                }
            } catch (e: Exception) {
                // Sometimes this can fail with invalid inputs
            }
        }

        return null
    }
    private fun resetTheta() {
        thetaX = 0f
        thetaY = 0f
        thetaZ = 0f
        ekf.reset()
        filteredRoll = 0f
        filteredPitch = 0f
        filteredYaw = 0f
        absoluteYaw = 0f
        isFirstReading = true
    }

    private fun toggleGyroscope() {
        isGyroscopeRunning = !isGyroscopeRunning
        if (isGyroscopeRunning) {
            startSensors()
        } else {
            stopSensors()
        }
    }

    private fun startSensors() {
        gyroscope?.let {
            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_NORMAL
            )
        }
        accelerometer?.let {
            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_NORMAL
            )
        }
        magnetometer?.let {
            sensorManager.registerListener(
                this,
                it,
                SensorManager.SENSOR_DELAY_NORMAL
            )
        }
        isFirstReading = true
    }

    private fun stopSensors() {
        gyroscope?.let {
            sensorManager.unregisterListener(this, it)
        }
        accelerometer?.let {
            sensorManager.unregisterListener(this, it)
        }
        magnetometer?.let {
            sensorManager.unregisterListener(this, it)
        }
    }

    override fun onResume() {
        super.onResume()
        if (isGyroscopeRunning) startSensors()
    }

    override fun onPause() {
        super.onPause()
        stopSensors()
    }

    private fun detectStationaryState(gyroX: Float, gyroY: Float, gyroZ: Float) {
        // Calculate magnitude of angular velocity
        val magnitude = sqrt(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ)
        // Add to recent measurements buffer
        recentGyroMagnitudes.add(magnitude)
        if (recentGyroMagnitudes.size > staticWindowSize) {
            recentGyroMagnitudes.removeFirst()
        }
        // Only check if we have enough samples
        if (recentGyroMagnitudes.size == staticWindowSize) {
            // Device is considered stationary if all recent gyro magnitudes are below threshold
            isStationary = recentGyroMagnitudes.all { it < movementThreshold }
        }
    }

    private fun updateBiasEstimates(gyroX: Float, gyroY: Float, gyroZ: Float) {
        if (isStationary) {
            // Update bias estimates with a low-pass filter when stationary
            if (biasEstimationCount < Constants.BIAS_ESTIMATION_COUNT_MAX) {  // Initial bias estimation
                gyroBiasX += gyroX / 100
                gyroBiasY += gyroY / 100
                gyroBiasZ += gyroZ / 100
                biasEstimationCount++
            } else {
                // Refine bias with low-pass filter
                gyroBiasX = (1 - biasUpdateRate) * gyroBiasX + biasUpdateRate * gyroX
                gyroBiasY = (1 - biasUpdateRate) * gyroBiasY + biasUpdateRate * gyroY
                gyroBiasZ = (1 - biasUpdateRate) * gyroBiasZ + biasUpdateRate * gyroZ
            }
        }
    }

    override fun onSensorChanged(event: SensorEvent?) {
        when (event?.sensor?.type) {
            Sensor.TYPE_GYROSCOPE -> {
                val currentTimestamp = System.currentTimeMillis()

                // Raw gyroscope readings
                val rawGyroX = event.values[0]
                val rawGyroY = event.values[1]
                val rawGyroZ = event.values[2]

                // 1. Detect if device is stationary
                detectStationaryState(rawGyroX, rawGyroY, rawGyroZ)

                // 2. Update bias estimates when stationary
                updateBiasEstimates(rawGyroX, rawGyroY, rawGyroZ)

                // 3. Apply bias correction to get corrected gyro values
                gyroscopeX = if (isStationary) 0f else rawGyroX - gyroBiasX
                gyroscopeY = if (isStationary) 0f else rawGyroY - gyroBiasY
                gyroscopeZ = if (isStationary) 0f else rawGyroZ - gyroBiasZ

                if (!isFirstReading) {
                    val deltaTime = (currentTimestamp - lastTimestamp) / Constants.MILLISECONDS_TO_SECONDS

                    // Standard integration for the ZUPT-corrected angles
                    thetaX += gyroscopeX * deltaTime
                    thetaY += gyroscopeY * deltaTime
                    thetaZ += gyroscopeZ * deltaTime

                    // Run EKF prediction with corrected gyro values
                    ekf.predict(gyroscopeX, gyroscopeY, gyroscopeZ, deltaTime)

                    // Get EKF-filtered angles
                    filteredRoll = ekf.getRoll()
                    filteredPitch = ekf.getPitch()
                    filteredYaw = absoluteYaw
                } else {
                    isFirstReading = false
                }

                lastTimestamp = currentTimestamp

                // Add to readings list
                gyroscopeReadings.add(
                    SensorReading(
                        timestamp = currentTimestamp,
                        gyro_x = gyroscopeX,
                        gyro_y = gyroscopeY,
                        gyro_z = gyroscopeZ,
                        accel_x = accelX,
                        accel_y = accelY,
                        accel_z = accelZ,
                        mag_x = magX,
                        mag_y = magY,
                        mag_z = magZ,
                        theta_x = thetaX,      // Raw integrated
                        theta_y = thetaY,
                        theta_z = thetaZ,
                        filtered_x = filteredRoll,  // EKF filtered
                        filtered_y = filteredPitch,
                        filtered_z = filteredYaw,
                        isStationary = isStationary
                    )
                )

                // Keep list size manageable
                if (gyroscopeReadings.size > maxDataPoints) {
                    gyroscopeReadings.removeAt(0)
                }
            }

            Sensor.TYPE_ACCELEROMETER -> {
                // Store accelerometer values
                accelX = event.values[0]
                accelY = event.values[1]
                accelZ = event.values[2]

                // Run EKF update with accelerometer data
                ekf.update(accelX, accelY, accelZ)
            }

            Sensor.TYPE_MAGNETIC_FIELD -> {
                // Store magnetometer values
                magX = event.values[0]
                magY = event.values[1]
                magZ = event.values[2]

                val orientation = calculateAbsoluteOrientation()
                if (orientation != null) {
                    // Orientation[0] is azimuth (yaw), but in Android's coordinate system
                    // We need to convert it to our coordinate system
                    absoluteYaw = orientation[0]

                    // Only update EKF with magnetometer if we're relatively stationary
                    // to avoid introducing noise during rapid movements
                    if (isStationary) {
                        ekf.updateWithYawMeasurement(absoluteYaw)
                        filteredYaw = ekf.getYaw()
                    }
                }
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        // Not needed for this example
    }
}


@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun MainScreen(
    gyroscopeX: Float,
    gyroscopeY: Float,
    gyroscopeZ: Float,
    accelX: Float,
    accelY: Float,
    accelZ: Float,
    thetaX: Float,
    thetaY: Float,
    thetaZ: Float,
    magX: Float,
    magY: Float,
    magZ: Float,
    filteredRoll: Float,
    filteredPitch: Float,
    isGyroscopeRunning: Boolean,
    isStationary: Boolean,
    gyroscopeReadings: List<SensorReading>,
    onGyroscopeToggle: () -> Unit,
    onResetTheta: () -> Unit,
    absoluteYaw: Float,
) {
    val navController = rememberNavController()

    NavHost(navController = navController, startDestination = "main") {
        composable("main") {
            MainContent(
                gyroscopeX = gyroscopeX,
                gyroscopeY = gyroscopeY,
                gyroscopeZ = gyroscopeZ,
                accelX = accelX,
                accelY = accelY,
                accelZ = accelZ,
                thetaX = thetaX,
                thetaY = thetaY,
                thetaZ = thetaZ,
                magX = magX,
                magY = magY,
                magZ = magZ,
                absoluteYaw = absoluteYaw,
                filteredRoll = filteredRoll,
                filteredPitch = filteredPitch,
                filteredYaw = absoluteYaw,
                isGyroscopeRunning = isGyroscopeRunning,
                isStationary = isStationary,
                onGyroscopeToggle = onGyroscopeToggle,
                onResetTheta = onResetTheta,
                gyroscopeReadings = gyroscopeReadings,
                navController = navController
            )
        }

        composable("graphs_screen") {
            GyroscopeGraphScreen(
                rawReadings = gyroscopeReadings,
                onBack = { navController.navigateUp() }
            )
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun MainContent(
    gyroscopeX: Float,
    gyroscopeY: Float,
    gyroscopeZ: Float,
    thetaX: Float,
    thetaY: Float,
    thetaZ: Float,
    isGyroscopeRunning: Boolean,
    isStationary: Boolean,
    accelX: Float,
    accelY: Float,
    accelZ: Float,
    magX: Float,
    magY: Float,
    magZ: Float,
    filteredRoll: Float,
    filteredPitch: Float,
    filteredYaw: Float,
    onGyroscopeToggle: () -> Unit,
    onResetTheta: () -> Unit,
    gyroscopeReadings: List<SensorReading>,
    navController: NavController,
    absoluteYaw: Float,
) {
    Column(modifier = Modifier.fillMaxSize()) {
        // Top App Bar
        TopAppBar(
            title = {
                Text(
                    text = "Gyroscope Monitor",
                    style = MaterialTheme.typography.titleLarge.copy(
                        fontWeight = FontWeight.SemiBold
                    ),
                    color = Color(0xFF2D223D)
                )
            },
            colors = TopAppBarDefaults.topAppBarColors(
                containerColor = Color(0xFFF6F2FA)
            )
        )

        // Content area - directly show SensorsTab
        Box(
            modifier = Modifier
                .weight(1f)
                .fillMaxWidth()
        ) {
            SensorsTab(
                gyroscopeX = gyroscopeX,
                gyroscopeY = gyroscopeY,
                gyroscopeZ = gyroscopeZ,
                thetaX = thetaX,
                thetaY = thetaY,
                thetaZ = thetaZ,
                isStationary = isStationary,
                accelX = accelX,
                accelY = accelY,
                accelZ = accelZ,
                magX = magX,
                magY = magY,
                magZ = magZ,
                filteredRoll = filteredRoll,
                filteredPitch = filteredPitch,
                filteredYaw = filteredYaw,
                isGyroscopeRunning = isGyroscopeRunning,
                onGyroscopeToggle = onGyroscopeToggle,
                onResetTheta = onResetTheta,
                navController = navController,
                absoluteYaw = absoluteYaw,
                gyroscopeReadings = gyroscopeReadings
            )
        }
    }
}