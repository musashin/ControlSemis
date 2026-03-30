#include "SensorManager.h"
#include <Arduino.h>

bool SensorManager::begin(int altitude_m) {
    Wire.begin();  // SDA = A4, SCL = A5 (default I2C pins on Arduino R4)

    if (!_scd30.begin(Wire)) {
        _healthy = false;
        #ifdef DEBUG
        Serial.println("[Sensor] SCD30 begin() failed!");
        #endif
        return false;
    }

    #ifdef DEBUG
    Serial.println("[Sensor] SCD30 detected");
    #endif

    _scd30.setAltitudeCompensation(static_cast<uint16_t>(altitude_m));
    #ifdef DEBUG
    Serial.print("[Sensor] Altitude compensation set to ");
    Serial.print(altitude_m);
    Serial.println(" m");
    #endif

    _scd30.setAutoSelfCalibration(true);
    #ifdef DEBUG
    Serial.println("[Sensor] Auto self-calibration enabled");
    #endif

    _scd30.setMeasurementInterval(2);  // measure every 2 s (minimum)
    #ifdef DEBUG
    Serial.println("[Sensor] Measurement interval: 2 s");
    #endif

    _healthy = true;
    return true;
}

SensorData SensorManager::read() {
    #ifdef DEBUG
    Serial.println("[Sensor] read() starting...");
    #endif

    SensorData data;

    if (!_healthy) {
        #ifdef DEBUG
        Serial.println("[Sensor] read() - sensor not healthy, returning invalid");
        #endif
        return data;  // valid = false
    }

    // Wait for a fresh measurement (SCD30 updates every 2 s; 5 s safety timeout).
    const unsigned long timeout = 5000UL;
    unsigned long start = millis();
    int waitCycles = 0;
    while (!_scd30.dataAvailable()) {
        if (millis() - start >= timeout) {
            _healthy = false;
            #ifdef DEBUG
            Serial.println("[Sensor] read() - timeout waiting for data!");
            #endif
            return data;  // valid = false
        }
        delay(100);
        waitCycles++;
    }

    #ifdef DEBUG
    Serial.print("[Sensor] Data available after ");
    Serial.print(waitCycles);
    Serial.println(" × 100ms");
    #endif

    data.temperature = _scd30.getTemperature();
    data.humidity    = _scd30.getHumidity();
    data.co2         = _scd30.getCO2();
    data.valid       = true;
    _healthy         = true;

    #ifdef DEBUG
    Serial.print("[Sensor] Read successful: ");
    Serial.print("T=");
    Serial.print(data.temperature, 1);
    Serial.print("°C, H=");
    Serial.print(data.humidity, 1);
    Serial.print("%, CO2=");
    Serial.print(data.co2, 0);
    Serial.println(" ppm");
    #endif

    return data;
}

