#pragma once
#include <ArduinoGraphics.h>
#include <Arduino_LED_Matrix.h>

enum class ControllerState {
    HEALTHY,        // MQTT + sensor OK   → static "OK"
    SENSOR_ERROR,   // MQTT OK, no sensor → scroll "SENS"
    COMM_FAULT      // WiFi/MQTT down     → scroll "COMMS ISSUE"
};

class DisplayManager {
public:
    void begin();

    // Render the state on the 12×8 LED matrix.
    // "OK" is a fast static update; error messages scroll (blocking, ~1-4 s).
    // Call only when state changes or on the 30-second refresh tick.
    void show(ControllerState state);

private:
    ArduinoLEDMatrix _matrix;

    void showStatic(const char* text);
    void showScrolling(const char* text);
};
