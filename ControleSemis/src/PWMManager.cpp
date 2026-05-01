#include "PWMManager.h"
#include "config.h"
#include <Arduino.h>

void PWMManager::begin() {
    pinMode(PWM_pin, OUTPUT);
    setDutyCycle(DEFAULT_PWM_DUTY_CYCLE);  // Start at configured default (safety)
    #ifdef DEBUG
    Serial.println("[PWM] Initialized on pin D9");
    Serial.print("[PWM] Initial duty cycle: ");
    Serial.print(_dutyCycle);
    Serial.println("%");
    #endif
}

void PWMManager::setDutyCycle(int percent) {
    // Clamp to 0-100
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    _dutyCycle = percent;

    // Map 0-100 to 0-255 (analogWrite range)
    int pwmValue = (percent * 255) / 100;
    analogWrite(PWM_pin, pwmValue);

    #ifdef DEBUG
    Serial.print("[PWM] Duty cycle set to ");
    Serial.print(percent);
    Serial.print("% (analogWrite value: ");
    Serial.print(pwmValue);
    Serial.println(")");
    #endif
}
