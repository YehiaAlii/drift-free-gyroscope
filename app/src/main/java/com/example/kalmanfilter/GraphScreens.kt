package com.example.kalmanfilter

import android.content.Context
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ArrowBack
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Text
import androidx.compose.material3.TopAppBar
import androidx.compose.material3.TopAppBarDefaults
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.compose.ui.viewinterop.AndroidView
import com.github.mikephil.charting.charts.LineChart
import com.github.mikephil.charting.components.Legend
import com.github.mikephil.charting.components.XAxis
import com.github.mikephil.charting.data.Entry
import com.github.mikephil.charting.data.LineData
import com.github.mikephil.charting.data.LineDataSet
import kotlin.math.PI

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun GyroscopeGraphScreen(
    rawReadings: List<SensorReading>,
    onBack: () -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(bottom = 12.dp)
    ) {
        // Top App Bar
        TopAppBar(
            title = {
                Text(
                    text = "Gyroscope Graphs",
                    style = MaterialTheme.typography.titleLarge.copy(
                        fontWeight = FontWeight.SemiBold
                    ),
                    color = Color(0xFF2D223D)
                )
            },
            navigationIcon = {
                IconButton(onClick = onBack) {
                    Icon(
                        imageVector = Icons.Filled.ArrowBack,
                        contentDescription = "Back",
                        tint = Color(0xFF6750A4)
                    )
                }
            },
            colors = TopAppBarDefaults.topAppBarColors(
                containerColor = Color(0xFFF6F2FA)
            )
        )
        // Graphs Content
        Column(
            modifier = Modifier
                .fillMaxSize()
                .verticalScroll(rememberScrollState())
        ) {

            Spacer(modifier = Modifier.height(16.dp))
            // Angular Velocity Graph
            GraphSection(
                title = "Angular Velocity",
                color = Color(0xFF6750A4)
            ) {
                AndroidView(
                    modifier = Modifier
                        .height(300.dp)
                        .fillMaxWidth()
                        .padding(bottom = 4.dp),
                    factory = { context -> createLineChart(context, "Angular Velocity (°/s)", -300f, 300f) },
                    update = { chart ->
                        val dataSets = listOf(
                            createDataSet(rawReadings.map { it.gyro_x * 180f / PI.toFloat() }, "ω X", Color.Red),
                            createDataSet(rawReadings.map { it.gyro_y * 180f / PI.toFloat() }, "ω Y", Color.Green),
                            createDataSet(rawReadings.map { it.gyro_z * 180f / PI.toFloat() }, "ω Z", Color.Blue)
                        )
                        chart.data = LineData(dataSets)
                        chart.invalidate()
                    }
                )
            }

            Spacer(modifier = Modifier.height(16.dp))

            // Theta (Angle) Graph
            GraphSection(
                title = "Integrated Angle θ (°)",
                color = Color(0xFF2E7D32)
            ) {
                AndroidView(
                    modifier = Modifier
                        .height(300.dp)
                        .fillMaxWidth()
                        .padding(bottom = 4.dp),
                    factory = { context -> createLineChart(context, "Angle", -180f, 180f) },
                    update = { chart ->
                        val dataSets = listOf(
                            createDataSet(rawReadings.map { it.theta_x * 180f / PI.toFloat() }, "θ X", Color.Red),
                            createDataSet(rawReadings.map { it.theta_y * 180f / PI.toFloat() }, "θ Y", Color.Green),
                            createDataSet(rawReadings.map { it.theta_z * 180f / PI.toFloat() }, "θ Z", Color.Blue)
                        )
                        chart.data = LineData(dataSets)
                        chart.invalidate()
                    }
                )
            }

            Spacer(modifier = Modifier.height(16.dp))

            // EKF Filtered Angles Graph
            GraphSection(
                title = "EKF Filtered Angles",
                color = Color(0xFF9C27B0)  // Purple
            ) {
                AndroidView(
                    modifier = Modifier
                        .height(300.dp)
                        .fillMaxWidth()
                        .padding(bottom = 4.dp),
                    factory = { context ->
                        // Use -180 to 180 degree range
                        createLineChart(context, "Filtered Angles", -180f, 180f)
                    },
                    update = { chart ->
                        val dataSets = listOf(
                            // Convert radians to degrees (multiply by 180/π)
                            createDataSet(
                                rawReadings.map { it.filtered_x * 180f / PI.toFloat() },
                                "Roll",
                                Color.Red
                            ),
                            createDataSet(
                                rawReadings.map { it.filtered_y * 180f / PI.toFloat() },
                                "Pitch",
                                Color.Green
                            ),
                            createDataSet(
                                rawReadings.map { it.filtered_z * 180f / PI.toFloat() },
                                "Yaw",
                                Color.Blue
                            )
                        )
                        chart.data = LineData(dataSets)
                        chart.invalidate()
                    }
                )
            }

        }
    }
}

@Composable
private fun GraphSection(
    title: String,
    color: Color,
    content: @Composable () -> Unit
) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        modifier = Modifier
            .padding(top = 4.dp, start = 12.dp, bottom = 4.dp)
    ) {
        Box(
            Modifier
                .width(6.dp)
                .height(28.dp)
                .background(color, shape = MaterialTheme.shapes.medium)
        )
        Spacer(Modifier.width(10.dp))
        Text(
            text = title,
            color = Color(0xFF22223B),
            fontWeight = FontWeight.Bold,
            fontSize = 18.sp,
            letterSpacing = 1.sp,
            modifier = Modifier
                .background(Color(0xFFF3F0FF), shape = MaterialTheme.shapes.medium)
                .padding(vertical = 4.dp, horizontal = 12.dp)
        )
    }
    content()
}

private fun createLineChart(context: Context, title: String, minY: Float, maxY: Float): LineChart {
    return LineChart(context).apply {
        setBackgroundColor(Color.Black.toArgb())
        description.isEnabled = false
        setTouchEnabled(false)
        isDragEnabled = false
        setScaleEnabled(false)
        setPinchZoom(false)

        xAxis.apply {
            textColor = Color.White.toArgb()
            setDrawGridLines(true)
            gridColor = Color.DarkGray.toArgb()
            position = XAxis.XAxisPosition.BOTTOM
            axisMinimum = 0f
            axisMaximum = 100f
            labelCount = 11
            setDrawLabels(true)
            granularity = 10f
        }

        axisLeft.apply {
            textColor = Color.White.toArgb()
            setDrawGridLines(true)
            gridColor = Color.DarkGray.toArgb()
            axisMinimum = minY
            axisMaximum = maxY
            labelCount = 11
            setDrawLabels(true)
            granularity = 1f
        }

        axisRight.isEnabled = false
        legend.apply {
            textColor = Color.White.toArgb()
            textSize = 12f
            form = Legend.LegendForm.LINE
        }
        isHighlightPerDragEnabled = false
        isHighlightPerTapEnabled = false
    }
}

private fun createDataSet(values: List<Float>, label: String, color: Color): LineDataSet {
    val entries = values.mapIndexed { index, value ->
        Entry(index.toFloat(), value)
    }

    return LineDataSet(entries, label).apply {
        this.color = color.toArgb()
        setDrawCircles(false)
        setDrawValues(false)
        lineWidth = 1.5f
        mode = LineDataSet.Mode.LINEAR
        setDrawHighlightIndicators(false)
        isHighlightEnabled = false
    }
}
