#pragma once

class PWMManager {
public:
    // Initialize PWM on pin D9 (PWM pin on Arduino UNO R4 WiFi)
    void begin();

    // Set PWM duty cycle (0-100)
    // Maps to analogWrite(0-255)
    void setDutyCycle(int percent);

    int getDutyCycle() const { return _dutyCycle; }

private:
    static const int PWM_pin = 9;  // D9 on Arduino R4 WiFi
    int _dutyCycle = 0;
};
