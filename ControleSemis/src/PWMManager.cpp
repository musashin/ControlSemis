#include "PWMManager.h"
#include "config.h"
#include <Arduino.h>
#include "pwm.h"

// ── PWM Frequency Configuration ──────────────────────────────────────────────
// Using Arduino's PwmOut class for 25kHz PWM to eliminate audible noise
// Default Arduino PWM is ~980 Hz (audible), 25kHz moves it above human hearing

PwmOut* pwmInstance = nullptr;

void PWMManager::begin() {
    // Clean up any existing instance
    if (pwmInstance != nullptr) {
        pwmInstance->end();
        delete pwmInstance;
        pwmInstance = nullptr;
    }

    // Create new PWM instance for pin D9
    pwmInstance = new PwmOut(PWM_pin);

    // Configure for 20kHz frequency with 0% initial duty cycle (conservative but still silent)
    // 20kHz is well above audible range (>20Hz) and reliable on RA4M1
    // begin(freq_hz, duty_percent)
    if (pwmInstance->begin(20000.0f, 0.0f)) {
        #ifdef DEBUG
        Serial.println("[PWM] Initialized on pin D9 with 20kHz frequency (silent operation)");
        #endif
        setDutyCycle(DEFAULT_PWM_DUTY_CYCLE);  // Start at configured default (safety)
    } else {
        #ifdef DEBUG
        Serial.println("[PWM] ERROR: Failed to initialize 25kHz PWM");
        #endif
    }
}

void PWMManager::setDutyCycle(int percent) {
    // Clamp to 0-100
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    _dutyCycle = percent;

    // Use Arduino's PWM API to set duty cycle percentage
    if (pwmInstance != nullptr) {
        pwmInstance->pulse_perc((float)percent);
    }

    #ifdef DEBUG
    Serial.print("[PWM] Duty cycle set to ");
    Serial.print(percent);
    Serial.println("%");
    #endif
}
