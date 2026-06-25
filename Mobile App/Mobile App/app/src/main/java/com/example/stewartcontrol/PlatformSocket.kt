package com.example.stewartcontrol

import kotlinx.coroutines.*
import org.java_websocket.client.WebSocketClient
import org.java_websocket.handshake.ServerHandshake
import org.json.JSONObject
import java.net.URI

/**
 * Thin WebSocket wrapper around java-websocket.
 * Reconnects automatically if the connection drops.
 *
 * Dependency to add in app/build.gradle:
 *   implementation 'org.java-websocket:Java-WebSocket:1.5.4'
 */
class PlatformSocket(
    private val host: String,
    private val port: Int = 8765,
    private val onStateChange: (State) -> Unit,
    private val onLatency: (Long) -> Unit,        // round-trip ms from ping/pong
) {
    enum class State { DISCONNECTED, CONNECTING, CONNECTED, ERROR }

    private var ws: WebSocketClient? = null
    private var pingJob: Job? = null
    private var pingTime: Long = 0L
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())

    private val uri get() = URI("ws://$host:$port")

    // ── Public API ───────────────────────────────────────────────────────────

    fun connect() {
        if (ws?.isOpen == true) return
        onStateChange(State.CONNECTING)
        buildClient().connectBlocking()  // blocks briefly — call from IO thread
    }

    fun disconnect() {
        pingJob?.cancel()
        ws?.closeBlocking()
        ws = null
        onStateChange(State.DISCONNECTED)
    }

    /** Send a realtime pose (fired at 50 Hz from sensor loop). */
    fun sendPose(x: Float, y: Float, z: Float, roll: Float, pitch: Float, yaw: Float) {
        if (ws?.isOpen != true) return
        val json = JSONObject().apply {
            put("type",  "rt")
            put("x",     x.toDouble())
            put("y",     y.toDouble())
            put("z",     z.toDouble())
            put("roll",  roll.toDouble())
            put("pitch", pitch.toDouble())
            put("yaw",   yaw.toDouble())
        }
        try { ws?.send(json.toString()) } catch (_: Exception) {}
    }

    fun sendHome()    = sendCommand("home")
    fun sendStop()    = sendCommand("stop")
    fun sendRelease() = sendCommand("release")

    val isConnected get() = ws?.isOpen == true

    // ── Private ──────────────────────────────────────────────────────────────

    private fun sendCommand(type: String) {
        if (ws?.isOpen != true) return
        try { ws?.send(JSONObject().put("type", type).toString()) } catch (_: Exception) {}
    }

    private fun buildClient(): WebSocketClient {
        val client = object : WebSocketClient(uri) {
            override fun onOpen(h: ServerHandshake) {
                onStateChange(State.CONNECTED)
                startPing()
            }
            override fun onMessage(msg: String) {
                try {
                    val j = JSONObject(msg)
                    if (j.optString("type") == "pong") {
                        onLatency(System.currentTimeMillis() - pingTime)
                    }
                } catch (_: Exception) {}
            }
            override fun onClose(code: Int, reason: String, remote: Boolean) {
                pingJob?.cancel()
                onStateChange(State.DISCONNECTED)
                // Auto-reconnect after 2 s
                scope.launch {
                    delay(2_000)
                    connect()
                }
            }
            override fun onError(ex: Exception) {
                onStateChange(State.ERROR)
            }
        }
        ws = client
        return client
    }

    private fun startPing() {
        pingJob?.cancel()
        pingJob = scope.launch {
            while (isActive) {
                delay(1_000)
                if (ws?.isOpen == true) {
                    pingTime = System.currentTimeMillis()
                    try { ws?.send(JSONObject().put("type", "ping").toString()) }
                    catch (_: Exception) {}
                }
            }
        }
    }
}
