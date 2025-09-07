package com.example.kalmanfilter

data class SensorReading(
    val timestamp: Long,
    val gyro_x: Float,
    val gyro_y: Float,
    val gyro_z: Float,
    val accel_x: Float = 0f,
    val accel_y: Float = 0f,
    val accel_z: Float = 0f,
    val mag_x: Float = 0f,
    val mag_y: Float = 0f,
    val mag_z: Float = 0f,
    val theta_x: Float = 0f,  // Integrated angles
    val theta_y: Float = 0f,
    val theta_z: Float = 0f,
    val filtered_x: Float = 0f, // EKF filtered angles
    val filtered_y: Float = 0f,
    val filtered_z: Float = 0f,
    val isStationary: Boolean = false
)
