package com.example.stewartcontrol

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.*
import android.view.WindowManager
import android.widget.*
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.SwitchCompat
import androidx.core.content.getSystemService
import kotlinx.coroutines.*

/**
 * Stewart Platform Gyroscopic Controller
 * =======================================
 * Reads phone IMU at 50 Hz, fuses gyro+accel through Madgwick filter,
 * maps resulting roll/pitch/yaw to the platform's coordinate space,
 * and streams "rt" commands to the RPi bridge over WebSocket.
 *
 * Z (heave) and sensitivity are adjusted via on-screen sliders.
 *
 * Layout file: res/layout/activity_main.xml
 */
class MainActivity : AppCompatActivity(), SensorEventListener {

    // ── Platform limits (must match bridge.py / Stewart.c) ───────────────────
    private val Z_HOME         = 125f
    private val Z_MIN          = 115f
    private val Z_MAX          = 140f
    private val LIMIT_ROLL     = 10f
    private val LIMIT_PITCH    = 10f
    private val LIMIT_YAW      = 15f
    private val LOOP_HZ        = 50

    // ── Sensor ────────────────────────────────────────────────────────────────
    private lateinit var sensorManager: SensorManager
    private var gyroSensor: Sensor?  = null
    private var accelSensor: Sensor? = null

    private val fusion = SensorFusion(beta = 0.1f)
    private var lastTimestamp = 0L

    // Latest raw sensor readings (written from sensor thread, read from loop)
    @Volatile private var gx = 0f; @Volatile private var gy = 0f; @Volatile private var gz = 0f
    @Volatile private var ax = 0f; @Volatile private var ay = 0f; @Volatile private var az = 0f
    @Volatile private var sensorReady = false

    // ── State ─────────────────────────────────────────────────────────────────
    private var sensitivity = 1.0f   // 0.1 – 2.0 multiplier from slider
    private var zOffset     = 0f     // mm offset from Z_HOME, controlled by slider
    private var lockYaw     = true   // yaw is relative — usually lock it for stability

    // ── WebSocket ─────────────────────────────────────────────────────────────
    private lateinit var socket: PlatformSocket
    private var loopJob: Job? = null
    private val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())

    // ── Views (set in onCreate, referenced in UI updates) ─────────────────────
    private lateinit var tvStatus:    TextView
    private lateinit var tvPose:      TextView
    private lateinit var tvLatency:   TextView
    private lateinit var etHost:      EditText
    private lateinit var btnConnect:  Button
    private lateinit var btnHome:     Button
    private lateinit var btnStop:     Button
    private lateinit var btnCalib:    Button
    private lateinit var sbSensitivity: SeekBar
    private lateinit var tvSensLabel: TextView
    private lateinit var sbZ:          SeekBar
    private lateinit var tvZLabel:     TextView
    private lateinit var swYaw:        SwitchCompat

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)
        setContentView(R.layout.activity_main)
        bindViews()
        setupSensors()
        setupUI()
    }

    override fun onResume() {
        super.onResume()
        registerSensors()
    }

    override fun onPause() {
        super.onPause()
        sensorManager.unregisterListener(this)
        loopJob?.cancel()
    }

    override fun onDestroy() {
        super.onDestroy()
        if (::socket.isInitialized) {
            runBlocking(Dispatchers.IO) { socket.sendHome(); delay(200); socket.disconnect() }
        }
        scope.cancel()
    }

    // ── View binding ──────────────────────────────────────────────────────────

    private fun bindViews() {
        tvStatus     = findViewById(R.id.tvStatus)
        tvPose       = findViewById(R.id.tvPose)
        tvLatency    = findViewById(R.id.tvLatency)
        etHost       = findViewById(R.id.etHost)
        btnConnect   = findViewById(R.id.btnConnect)
        btnHome      = findViewById(R.id.btnHome)
        btnStop      = findViewById(R.id.btnStop)
        btnCalib     = findViewById(R.id.btnCalibrate)
        sbSensitivity= findViewById(R.id.sbSensitivity)
        tvSensLabel  = findViewById(R.id.tvSensLabel)
        sbZ          = findViewById(R.id.sbZ)
        tvZLabel     = findViewById(R.id.tvZLabel)
        swYaw        = findViewById(R.id.swYaw)
    }

    // ── Sensor setup ──────────────────────────────────────────────────────────

    private fun setupSensors() {
        sensorManager = getSystemService()!!
        gyroSensor    = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        accelSensor   = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
    }

    private fun registerSensors() {
        val rate = SensorManager.SENSOR_DELAY_FASTEST
        gyroSensor?.let  { sensorManager.registerListener(this, it, rate) }
        accelSensor?.let { sensorManager.registerListener(this, it, rate) }
    }

    // ── SensorEventListener ───────────────────────────────────────────────────

    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_GYROSCOPE -> {
                gx = event.values[0]; gy = event.values[1]; gz = event.values[2]
                val now = event.timestamp
                if (lastTimestamp != 0L) {
                    val dt = (now - lastTimestamp) * 1e-9f
                    fusion.update(gx, gy, gz, ax, ay, az, dt)
                    sensorReady = true
                }
                lastTimestamp = now
            }
            Sensor.TYPE_ACCELEROMETER -> {
                ax = event.values[0]; ay = event.values[1]; az = event.values[2]
            }
        }
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}

    // ── UI setup ──────────────────────────────────────────────────────────────

    private fun setupUI() {
        // Sensitivity slider: 0–100 mapped to 0.1–2.0
        sbSensitivity.max = 100
        sbSensitivity.progress = 47   // ~1.0 default
        sbSensitivity.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(sb: SeekBar, p: Int, fromUser: Boolean) {
                sensitivity = 0.1f + p / 100f * 1.9f
                tvSensLabel.text = "Sensitivity: ${"%.1f".format(sensitivity)}×"
            }
            override fun onStartTrackingTouch(sb: SeekBar) {}
            override fun onStopTrackingTouch(sb: SeekBar) {}
        })
        tvSensLabel.text = "Sensitivity: 1.0×"

        // Z (heave) slider: 0–100 mapped to Z_MIN..Z_MAX
        sbZ.max = 100
        sbZ.progress = 40   // Z_HOME default (~125)
        sbZ.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(sb: SeekBar, p: Int, fromUser: Boolean) {
                val z = Z_MIN + p / 100f * (Z_MAX - Z_MIN)
                zOffset = z - Z_HOME
                tvZLabel.text = "Height: ${"%.1f".format(z)} mm"
            }
            override fun onStartTrackingTouch(sb: SeekBar) {}
            override fun onStopTrackingTouch(sb: SeekBar) {}
        })
        tvZLabel.text = "Height: ${"%.1f".format(Z_HOME)} mm"

        swYaw.isChecked = !lockYaw
        swYaw.setOnCheckedChangeListener { _, checked -> lockYaw = !checked }

        btnConnect.setOnClickListener { toggleConnection() }
        btnHome.setOnClickListener    { if (::socket.isInitialized) socket.sendHome() }
        btnStop.setOnClickListener    { if (::socket.isInitialized) socket.sendStop() }
        btnCalib.setOnClickListener   { fusion.reset(); showToast("Calibrated — hold phone level") }
    }

    // ── Connect / Disconnect ──────────────────────────────────────────────────

    private fun toggleConnection() {
        if (::socket.isInitialized && socket.isConnected) {
            loopJob?.cancel()
            scope.launch(Dispatchers.IO) { socket.sendHome(); delay(300); socket.disconnect() }
            btnConnect.text = "Connect"
            setStatus("Disconnected")
        } else {
            val host = etHost.text.toString().trim().ifEmpty { "192.168.1.100" }
            buildSocket(host)
            scope.launch(Dispatchers.IO) { socket.connect() }
            btnConnect.text = "Disconnect"
            setStatus("Connecting…")
        }
    }

    private fun buildSocket(host: String) {
        socket = PlatformSocket(
            host        = host,
            port        = 8765,
            onStateChange = { state ->
                runOnUiThread {
                    when (state) {
                        PlatformSocket.State.CONNECTED    -> {
                            setStatus("Connected")
                            btnConnect.text = "Disconnect"
                            startSendLoop()
                        }
                        PlatformSocket.State.DISCONNECTED -> {
                            setStatus("Disconnected")
                            btnConnect.text = "Connect"
                            loopJob?.cancel()
                        }
                        PlatformSocket.State.CONNECTING   -> setStatus("Connecting…")
                        PlatformSocket.State.ERROR        -> setStatus("Connection error")
                    }
                }
            },
            onLatency = { ms ->
                runOnUiThread { tvLatency.text = "Latency: ${ms} ms" }
            }
        )
    }

    // ── 50 Hz send loop ───────────────────────────────────────────────────────

    private fun startSendLoop() {
        loopJob?.cancel()
        loopJob = scope.launch {
            val periodMs = 1000L / LOOP_HZ
            while (isActive) {
                val t0 = System.currentTimeMillis()

                if (sensorReady && socket.isConnected) {
                    val (roll, pitch, yaw) = fusion.getEulerDeg()

                    // Scale by sensitivity and clamp
                    val r = (roll  * sensitivity).coerceIn(-LIMIT_ROLL,  LIMIT_ROLL)
                    val p = (pitch * sensitivity).coerceIn(-LIMIT_PITCH, LIMIT_PITCH)
                    val y = if (lockYaw) 0f
                    else (yaw * sensitivity).coerceIn(-LIMIT_YAW, LIMIT_YAW)
                    val z = (Z_HOME + zOffset).coerceIn(Z_MIN, Z_MAX)

                    socket.sendPose(x = 0f, y = 0f, z = z, roll = r, pitch = p, yaw = y)

                    runOnUiThread {
                        tvPose.text = "R: ${"%.1f".format(r)}°  P: ${"%.1f".format(p)}°  " +
                                "Y: ${"%.1f".format(y)}°  Z: ${"%.1f".format(z)} mm"
                    }
                }

                val elapsed = System.currentTimeMillis() - t0
                val sleep   = periodMs - elapsed
                if (sleep > 0) delay(sleep)
            }
        }
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private fun setStatus(msg: String) { tvStatus.text = msg }

    private fun showToast(msg: String) {
        Toast.makeText(this, msg, Toast.LENGTH_SHORT).show()
    }
}
