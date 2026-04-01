#include <Arduino.h>
#include "config.h"
#include "SensorManager.h"
#include "MQTTManager.h"
#include "DisplayManager.h"
#include "PWMManager.h"

// ── Scheduling intervals ──────────────────────────────────────────────────────
// Defined in config.h (adjust in config.h for testing)

// ── Subsystem instances ───────────────────────────────────────────────────────
static SensorManager  sensors;
static PWMManager     pwm;
static MQTTManager    mqtt(WIFI_SSID, WIFI_PASSWORD,
                           MQTT_BROKER, MQTT_PORT,
                           MQTT_USER, MQTT_PASSWORD,
                           MQTT_CLIENT_ID,
                           &pwm);
static DisplayManager display;

// ── Controller state ──────────────────────────────────────────────────────────
static ControllerState state = ControllerState::COMM_FAULT;

// ── Timestamps ────────────────────────────────────────────────────────────────
static unsigned long lastSensorMs     = 0;
static unsigned long lastControllerMs = 0;
static unsigned long lastDisplayMs    = 0;
static unsigned long lastHeartbeatMs  = 0;
static bool          displayNeedsUpdate = false;

static float lastSensorAirTempC = NAN;
static unsigned long lastControllerTempWarnMs = 0;

// ── Internal temperature (RA4M1 on Arduino UNO R4 WiFi) ──────────────────────
// Reads the built-in temperature sensor via direct register access.
// Formula: T(°C) = (1.394 − V) / 0.00435 + 25, where V = raw × 3.3 / 4096.
// Note: the raw offset may vary ±5 °C between chips; useful as a relative trend.
// Cached controller temperature to avoid publishing spurious values during sensor warmup.
static bool controllerTempIsValid = false;
static float controllerTempCache = 0.0f;

static float readControllerTemperature() {
    const float minTempC = -40.0f;
    const float maxTempC = 120.0f;
    const int samples = 3;
    const int sampleDelayMs = 20;

    // Warm up ADC path with one discarded conversion (helps remove first-cycle bias).
    R_ADC0->ADEXICR_b.TSSA = 1;
    R_ADC0->ADCSR_b.ADST = 1;
    while (R_ADC0->ADCSR_b.ADST);
    delay(sampleDelayMs);

    float sum = 0.0f;
    int validCount = 0;

    for (int i = 0; i < samples; ++i) {
        R_ADC0->ADEXICR_b.TSSA = 1;   // connect internal temp sensor to ADC
        R_ADC0->ADCSR_b.ADST   = 1;   // start single conversion
        while (R_ADC0->ADCSR_b.ADST); // wait for completion

        uint16_t raw = R_ADC0->ADTSDR;
        if (raw == 0 || raw >= 4095) {
            #ifdef DEBUG
            Serial.print("[Controller] Invalid ADC value: ");
            Serial.println(raw);
            #endif
            delay(sampleDelayMs);
            continue;
        }

        float v = raw * (3.3f / 4096.0f);
        float t = (1.394f - v) / 0.00435f + 25.0f;

        if (t < minTempC || t > maxTempC) {
            #ifdef DEBUG
            Serial.print("[Controller] Temp out of limits: ");
            Serial.println(t, 1);
            #endif
            delay(sampleDelayMs);
            continue;
        }

        sum += t;
        validCount++;
        delay(sampleDelayMs);
    }

    if (validCount == 0) {
        #ifdef DEBUG
        Serial.println("[Controller] No valid temperature sampling");
        #endif
        return NAN;
    }

    return sum / validCount;
}

// ── Task: check connection health and attempt reconnection ───────────────────
static void runHeartbeatTask() {
    #ifdef DEBUG
    Serial.println("\n[Loop:Heartbeat] ─────────────────────────────────────────");
    #endif

    bool wasConnected = (state != ControllerState::COMM_FAULT);
    bool isConnected = mqtt.isConnected();

    #ifdef DEBUG
    Serial.print("[Loop:Heartbeat] Current MQTT status: ");
    Serial.println(isConnected ? "connected" : "disconnected");
    #endif

    // Try to reconnect if we've lost connection
    if (!isConnected) {
        #ifdef DEBUG
        Serial.println("[Loop:Heartbeat] Attempting reconnection...");
        #endif
        isConnected = mqtt.connect();
        #ifdef DEBUG
        Serial.print("[Loop:Heartbeat] Reconnection result: ");
        Serial.println(isConnected ? "SUCCESS" : "FAILED");
        #endif
    }

    ControllerState newState;
    if (!isConnected) {
        newState = ControllerState::COMM_FAULT;
        // Safety: reset PWM to 0% when connection is lost
        if (wasConnected && pwm.getDutyCycle() != 0) {
            #ifdef DEBUG
            Serial.println("[Loop:Heartbeat] ⚠ Connection lost — resetting PWM to 0% (safety)");
            #endif
            pwm.setDutyCycle(0);
        }
    } else if (state == ControllerState::SENSOR_ERROR && !sensors.isHealthy()) {
        // Stay in sensor error if sensor is still unhealthy
        newState = ControllerState::SENSOR_ERROR;
    } else if (isConnected && sensors.isHealthy()) {
        newState = ControllerState::HEALTHY;
    } else {
        newState = state;  // maintain current state
    }

    // Flag display for update if state changed
    if (newState != state) {
        state = newState;
        displayNeedsUpdate = true;

        // Log state transitions
        const char* stateStr = (state == ControllerState::HEALTHY)      ? "HEALTHY"
                             : (state == ControllerState::SENSOR_ERROR) ? "SENSOR_ERROR"
                             : "COMM_FAULT";
        #ifdef DEBUG
        Serial.print("[Loop:Heartbeat] ⚠ STATE CHANGE → ");
        Serial.println(stateStr);
        #endif
    }
}

// ── Task: read sensor and publish air-quality data ────────────────────────────
static void runSensorTask() {
    #ifdef DEBUG
    Serial.println("\n[Loop:Sensor] ───────────────────────────────────────────────");
    #endif

    // Try to establish/maintain connection
    bool isConnected = mqtt.isConnected();
    #ifdef DEBUG
    Serial.print("[Loop:Sensor] MQTT status: ");
    Serial.println(isConnected ? "connected" : "disconnected");
    #endif

    if (!isConnected) {
        #ifdef DEBUG
        Serial.println("[Loop:Sensor] Attempting MQTT reconnect...");
        #endif
        isConnected = mqtt.connect();
        #ifdef DEBUG
        Serial.print("[Loop:Sensor] MQTT reconnect result: ");
        Serial.println(isConnected ? "SUCCESS" : "FAILED");
        #endif
    }

    ControllerState newState;
    if (!isConnected) {
        newState = ControllerState::COMM_FAULT;
        #ifdef DEBUG
        Serial.println("[Loop:Sensor] No MQTT connection → COMM_FAULT");
        #endif
    } else {
        #ifdef DEBUG
        Serial.println("[Loop:Sensor] Reading sensor...");
        #endif
        SensorData data = sensors.read();

        if (!data.valid) {
            newState = ControllerState::SENSOR_ERROR;
            #ifdef DEBUG
            Serial.println("[Loop:Sensor] Sensor read failed → SENSOR_ERROR");
            Serial.println("[Loop:Sensor] Publishing null values");
            #endif
        } else {
            newState = ControllerState::HEALTHY;
            lastSensorAirTempC = data.temperature;
            #ifdef DEBUG
            Serial.print("[Loop:Sensor] ✓ Data valid → HEALTHY");
            Serial.print(" (T=");
            Serial.print(data.temperature, 1);
            Serial.print("°C, H=");
            Serial.print(data.humidity, 1);
            Serial.print("%, CO2=");
            Serial.print(data.co2, 0);
            Serial.println(" ppm)");
            Serial.println("[Loop:Sensor] Publishing sensor data...");
            #endif
        }
        mqtt.publishSensorData(data);  // publishes "null" when !data.valid
    }

    if (newState != state) {
        state = newState;
        displayNeedsUpdate = true;
        #ifdef DEBUG
        Serial.print("[Loop:Sensor] ⚠ STATE CHANGE → ");
        Serial.println((state == ControllerState::HEALTHY)      ? "HEALTHY"
                     : (state == ControllerState::SENSOR_ERROR) ? "SENSOR_ERROR"
                     : "COMM_FAULT");
        #endif
    }
}

// ── Task: publish controller temperature and health status ────────────────────
static void runControllerTask() {
    #ifdef DEBUG
    Serial.println("\n[Loop:Controller] ───────────────────────────────────────────");
    #endif

    if (state == ControllerState::COMM_FAULT) {
        #ifdef DEBUG
        Serial.println("[Loop:Controller] State is COMM_FAULT, skipping publish");
        #endif
        return;  // nothing to send
    }

    bool commOk = mqtt.isConnected() || mqtt.connect();
    if (!commOk) {
        state = ControllerState::COMM_FAULT;
        displayNeedsUpdate = true;
        #ifdef DEBUG
        Serial.println("[Loop:Controller] Connection lost → COMM_FAULT");
        #endif
        return;
    }

    #ifdef DEBUG
    Serial.println("[Loop:Controller] Publishing controller data...");
    #endif

    float controllerTemp = readControllerTemperature();

    if (isnan(controllerTemp)) {
        if (!controllerTempIsValid) {
            // Fallback to last air temperature if available; avoids broadcasting 0.0 as control temperature.
            if (!isnan(lastSensorAirTempC)) {
                controllerTemp = lastSensorAirTempC;
                controllerTempIsValid = true;
                controllerTempCache = controllerTemp;
                #ifdef DEBUG
                Serial.print("[Loop:Controller] Using air-temp fallback for controller temp: ");
                Serial.print(controllerTemp, 1);
                Serial.println("°C");
                #endif
            } else {
                unsigned long now = millis();
                if (now - lastControllerTempWarnMs >= 15000UL) {
                    #ifdef DEBUG
                    Serial.println("[Loop:Controller] No valid controller temperature yet; skipping publish");
                    #endif
                    lastControllerTempWarnMs = now;
                }
                return;
            }
        } else {
            controllerTemp = controllerTempCache;
            unsigned long now = millis();
            if (now - lastControllerTempWarnMs >= 30000UL) {
                #ifdef DEBUG
                Serial.print("[Loop:Controller] Using cached controller temp: ");
                Serial.print(controllerTemp, 1);
                Serial.println("°C");
                #endif
                lastControllerTempWarnMs = now;
            }
        }
    } else {
        controllerTempIsValid = true;
        controllerTempCache = controllerTemp;
        #ifdef DEBUG
        Serial.print("[Loop:Controller] Controller temp: ");
        Serial.print(controllerTemp, 1);
        Serial.println("°C");
        #endif
    }

    mqtt.publishControllerTemp(controllerTemp);


    const char* statusStr = (state == ControllerState::HEALTHY)
                            ? "OK"
                            : "SENSOR FAULT";
    mqtt.publishControllerStatus(statusStr);

    #ifdef DEBUG
    Serial.print("[Loop:Controller] Status: ");
    Serial.println(statusStr);
    #endif
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);  // wait for serial to stabilize

    #ifdef DEBUG
    Serial.println("\n\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║           CONTROL SEMIS - ARDUINO R4 WIFI              ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");
    Serial.println("🔧 DEBUG BUILD - verbose logging enabled");
    Serial.println("\n▶ Initializing subsystems...\n");
    #endif

    display.begin();
    #ifdef DEBUG
    Serial.println("✓ Display initialized");
    #endif

    display.show(ControllerState::COMM_FAULT);

    // ─ PWM ─────────────────────────────────────────────────────────────────────
    #ifdef DEBUG
    Serial.println("\n▸ Initializing PWM controller...");
    #endif
    pwm.begin();
    #ifdef DEBUG
    Serial.println("✓ PWM initialized (pin D9, duty cycle 0%)");
    #endif

    // ─ Sensor ─────────────────────────────────────────────────────────────────
    #ifdef DEBUG
    Serial.println("\n▸ Initializing SCD30 sensor...");
    #endif
    if (!sensors.begin(ALTITUDE_M)) {
        #ifdef DEBUG
        Serial.println("✗ SCD30 initialization FAILED");
        #endif
    } else {
        #ifdef DEBUG
        Serial.println("✓ SCD30 sensor initialized");
        Serial.print("  Altitude compensation: ");
        Serial.print(ALTITUDE_M);
        Serial.println(" m (Montreal)");
        Serial.println("  Auto self-calibration: enabled");
        #endif
    }

    #ifdef DEBUG
    Serial.print("  Sensor health: ");
    if (sensors.isHealthy()) {
        Serial.println("✓ HEALTHY");
    } else {
        Serial.println("✗ FAULT");
    }
    #endif

    // ─ MQTT ────────────────────────────────────────────────────────────────────
    #ifdef DEBUG
    Serial.println("\n▸ Attempting MQTT connection...");
    #endif
    if (mqtt.connect()) {
        #ifdef DEBUG
        Serial.println("✓ MQTT connection established");
        #endif
        state = sensors.isHealthy() ? ControllerState::HEALTHY
                                    : ControllerState::SENSOR_ERROR;
    } else {
        #ifdef DEBUG
        Serial.println("✗ MQTT connection FAILED");
        #endif
        state = ControllerState::COMM_FAULT;
    }

    // ─ Initial state ──────────────────────────────────────────────────────────
    #ifdef DEBUG
    Serial.print("\n▶ Initial state: ");
    if (state == ControllerState::HEALTHY) {
        Serial.println("✓ HEALTHY");
    } else if (state == ControllerState::SENSOR_ERROR) {
        Serial.println("⚠ SENSOR_ERROR");
    } else {
        Serial.println("✗ COMM_FAULT");
    }
    #endif

    display.show(state);

    // ─ Initial sensor & controller tasks ──────────────────────────────────────
    #ifdef DEBUG
    Serial.println("\n▶ Running initial tasks...");
    #endif
    runSensorTask();
    runControllerTask();
    display.show(state);

    // ─ Setup timestamps ──────────────────────────────────────────────────────
    lastSensorMs     = millis();
    lastControllerMs = millis();
    lastDisplayMs    = millis();
    lastHeartbeatMs  = millis();

    #ifdef DEBUG
    Serial.println("\n═══════════════════════════════════════════════════════");
    Serial.println("Setup complete. Entering main loop.\n");
    #endif
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
    #ifdef DEBUG
    unsigned long loopStart = millis();
    #endif

    mqtt.poll();

    unsigned long now = millis();

    // Connection health check — runs every 30 s, allows fast reconnection on fault
    if (now - lastHeartbeatMs >= HEARTBEAT_CHECK_MS) {
        lastHeartbeatMs = now;
        runHeartbeatTask();
    }

    if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
        lastSensorMs = now;
        runSensorTask();
    }

    if (now - lastControllerMs >= CONTROLLER_INTERVAL_MS) {
        lastControllerMs = now;
        runControllerTask();
    }

    // Update display on state change or on refresh interval
    if (displayNeedsUpdate || (now - lastDisplayMs >= DISPLAY_REFRESH_MS)) {
        lastDisplayMs = now;
        displayNeedsUpdate = false;
        display.show(state);
        #ifdef DEBUG
        Serial.println("[Loop] Display updated");
        #endif
    }

    #ifdef DEBUG
    unsigned long loopDuration = millis() - loopStart;
    if (loopDuration > 100) {  // log slow loops
        Serial.print("[Loop] ⏱ Loop took ");
        Serial.print(loopDuration);
        Serial.println(" ms");
    }
    #endif
}
