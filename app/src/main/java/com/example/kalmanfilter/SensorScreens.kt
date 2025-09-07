package com.example.kalmanfilter

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.CardDefaults
import androidx.compose.material3.ElevatedCard
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.navigation.NavController
import kotlin.math.PI

@Composable
fun SensorsTab(
    gyroscopeX: Float,
    gyroscopeY: Float,
    gyroscopeZ: Float,
    thetaX: Float,
    thetaY: Float,
    thetaZ: Float,
    accelX: Float,
    accelY: Float,
    accelZ: Float,
    magX: Float,
    magY: Float,
    magZ: Float,
    filteredRoll: Float,
    filteredPitch: Float,
    filteredYaw: Float,
    isStationary: Boolean,
    isGyroscopeRunning: Boolean,
    onGyroscopeToggle: () -> Unit,
    onResetTheta: () -> Unit,
    navController: NavController,
    absoluteYaw: Float,
    gyroscopeReadings: List<SensorReading>
) {
    Box(
        modifier = Modifier
            .fillMaxSize(),
        contentAlignment = Alignment.Center
    ) {
        SensorCard(
            gyroscopeX = gyroscopeX,
            gyroscopeY = gyroscopeY,
            gyroscopeZ = gyroscopeZ,
            thetaX = thetaX,
            thetaY = thetaY,
            thetaZ = thetaZ,
            filteredRoll = filteredRoll,
            filteredPitch = filteredPitch,
            filteredYaw = filteredYaw,
            isStationary = isStationary,
            isRunning = isGyroscopeRunning,
            accelX = accelX,
            accelY = accelY,
            accelZ = accelZ,
            magX = magX,
            magY = magY,
            magZ = magZ,
            absoluteYaw = absoluteYaw,
            onToggle = onGyroscopeToggle,
            onResetTheta = onResetTheta,
            onViewGraph = { navController.navigate("graphs_screen") },
            gyroscopeReadings = gyroscopeReadings
        )
    }
}

@Composable
private fun SectionHeader(text: String, color: Color = Color.Gray) {
    Text(
        text = text,
        style = MaterialTheme.typography.labelLarge,
        color = color,
        modifier = Modifier.padding(bottom = 4.dp)
    )
}


@Composable
private fun ValuesRowBox(
    x: Float,
    y: Float,
    z: Float,
    textColor: Color,
    accentColor: Color
) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .background(accentColor, shape = MaterialTheme.shapes.medium)
            .padding(vertical = 10.dp, horizontal = 0.dp),
        horizontalArrangement = Arrangement.SpaceEvenly,
        verticalAlignment = Alignment.CenterVertically
    ) {
        ValueColumn("X", x, textColor)
        ValueColumn("Y", y, textColor)
        ValueColumn("Z", z, textColor)
    }
}

@Composable
private fun ValueColumn(label: String, value: Float, textColor: Color) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(
            label,
            style = MaterialTheme.typography.labelLarge,
            color = textColor.copy(alpha = 0.8f)
        )
        Text(
            "%.4f".format(value),
            style = MaterialTheme.typography.bodyLarge.copy(fontWeight = FontWeight.SemiBold),
            color = textColor
        )
    }
}

@Composable
fun SensorCard(
    gyroscopeX: Float,
    gyroscopeY: Float,
    gyroscopeZ: Float,
    thetaX: Float,
    thetaY: Float,
    thetaZ: Float,
    accelX: Float,
    accelY: Float,
    accelZ: Float,
    magX: Float,
    magY: Float,
    magZ: Float,
    filteredRoll: Float,
    filteredPitch: Float,
    filteredYaw: Float,
    absoluteYaw: Float,
    isRunning: Boolean,
    isStationary: Boolean,
    onToggle: () -> Unit,
    onResetTheta: () -> Unit,
    onViewGraph: () -> Unit,
    gyroscopeReadings: List<SensorReading>,
) {
    ElevatedCard(
        modifier = Modifier
            .fillMaxWidth()
            .padding(horizontal = 12.dp, vertical = 8.dp),
        elevation = CardDefaults.cardElevation(defaultElevation = 8.dp),
        shape = MaterialTheme.shapes.large
    ) {
        Column(
            modifier = Modifier
                .fillMaxWidth()
                .verticalScroll(rememberScrollState())
                .padding(20.dp)
        ) {

            // Title and graph button
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.SpaceBetween,
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Gyroscope",
                    style = MaterialTheme.typography.titleLarge.copy(fontWeight = FontWeight.Bold)
                )
                IconButton(onClick = onViewGraph) {
                    Icon(
                        painter = painterResource(R.drawable.chart),
                        contentDescription = "View Graph",
                        modifier = Modifier.size(32.dp).padding(3.dp),
                        tint = Color.Unspecified
                    )
                }
            }

            Spacer(Modifier.height(12.dp))

            // Gyroscope data section
            SectionHeader(text = "Angular Velocity (°/s)", color = Color(0xFF333333))
            ValuesRowBox(
                x = gyroscopeX * 180f / PI.toFloat(),
                y = gyroscopeY * 180f / PI.toFloat(),
                z = gyroscopeZ * 180f / PI.toFloat(),
                textColor = Color(0xFF2D223D),
                accentColor = Color(0xFFD6D5FE)
            )

            Spacer(modifier = Modifier.height(16.dp))

            // Angle in degrees
            SectionHeader(text = "Integrated Angle θ (degrees)", color = Color(0xFF333333))
            ValuesRowBox(
                x = thetaX * 180f / PI.toFloat(),
                y = thetaY * 180f / PI.toFloat(),
                z = thetaZ * 180f / PI.toFloat(),
                textColor = Color(0xFF2D223D),
                accentColor = Color(0xFFFFF2CC)
            )
            Spacer(modifier = Modifier.height(18.dp))

            SectionHeader(text = "EKF Filtered Angles (degrees)", color = Color(0xFF333333))
            ValuesRowBox(
                x = filteredRoll * 180f / PI.toFloat(),
                y = filteredPitch * 180f / PI.toFloat(),
                z = filteredYaw * 180f / PI.toFloat(),
                textColor = Color(0xFF2D223D),
                accentColor = Color(0xFFE1D5E7)  // Light purple
            )

            Spacer(modifier = Modifier.height(18.dp))

            Row(
                modifier = Modifier
                    .fillMaxWidth()
                    .padding(vertical = 8.dp),
                verticalAlignment = Alignment.CenterVertically
            ) {
                Text(
                    text = "Status: ",
                    style = MaterialTheme.typography.bodyLarge,
                    fontWeight = FontWeight.Medium
                )

                Box(
                    modifier = Modifier
                        .size(12.dp)
                        .background(
                            color = if (isStationary) Color.Green else Color.Red,
                            shape = CircleShape
                        )
                )

                Spacer(modifier = Modifier.width(8.dp))

                Text(
                    text = if (isStationary) "Stationary" else "Moving",
                    style = MaterialTheme.typography.bodyLarge
                )
            }

            Spacer(modifier = Modifier.height(18.dp))

            // Button row
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                // Pause/Resume Button
                Button(
                    onClick = onToggle,
                    modifier = Modifier
                        .weight(1f)
                        .height(48.dp),
                    shape = MaterialTheme.shapes.large,
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (isRunning) Color(0xFF6F45D3) else Color(0xFF9D9D9D)
                    )
                ) {
                    Text(
                        text = if (isRunning) "Pause" else "Resume",
                        color = Color.White,
                        style = MaterialTheme.typography.titleMedium,
                        fontWeight = FontWeight.Medium
                    )
                }

                // Reset Theta Button
                Button(
                    onClick = onResetTheta,
                    modifier = Modifier
                        .weight(1f)
                        .height(48.dp),
                    shape = MaterialTheme.shapes.large,
                    colors = ButtonDefaults.buttonColors(
                        containerColor = Color(0xFFE74C3C)
                    )
                ) {
                    Text(
                        text = "Reset θ",
                        color = Color.White,
                        style = MaterialTheme.typography.titleMedium,
                        fontWeight = FontWeight.Medium
                    )
                }
            }
        }
    }
}
