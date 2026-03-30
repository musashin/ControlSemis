#pragma once
#include <Wire.h>
#include <SparkFun_SCD30_Arduino_Library.h>

struct SensorData {
    float temperature = 0.0f;
    float humidity    = 0.0f;
    float co2         = 0.0f;
    bool  valid       = false;
};

class SensorManager {
public:
    // Initialise the SCD30. altitude_m is used for barometric compensation.
    // Returns true on success.
    bool begin(int altitude_m);

    // Blocking read (waits up to 5 s for data). Sets isHealthy() accordingly.
    SensorData read();

    bool isHealthy() const { return _healthy; }

private:
    SCD30 _scd30;
    bool  _healthy = false;
};
