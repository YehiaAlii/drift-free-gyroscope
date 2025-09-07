package com.example.kalmanfilter

object Constants {
    // ZUPT related constants
    const val MOVEMENT_THRESHOLD = 0.03f
    const val STATIC_WINDOW_SIZE = 7
    const val BIAS_UPDATE_RATE = 0.01f
    const val BIAS_ESTIMATION_COUNT_MAX = 100

    // Data management
    const val MAX_DATA_POINTS = 100

    // Time conversion
    const val MILLISECONDS_TO_SECONDS = 1000.0f

    // Permissions
    const val STORAGE_PERMISSION_CODE = 1001

    // UI Colors
    const val APP_BAR_BACKGROUND_COLOR = 0xFFF6F2FA
    const val APP_BAR_TEXT_COLOR = 0xFF2D223D
}