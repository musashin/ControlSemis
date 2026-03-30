#pragma once
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include "SensorManager.h"

class PWMManager;  // forward declaration

class MQTTManager {
public:
    MQTTManager(const char* ssid,     const char* wifiPass,
                const char* broker,   int         port,
                const char* user,     const char* mqttPass,
                const char* clientId,
                PWMManager* pwmManager = nullptr);

    // Attempt WiFi + MQTT connection. Returns true when both are up.
    bool connect();

    bool isConnected();

    // Must be called from loop() to service the MQTT keepalive and process messages.
    void poll();

    // Publish air-quality data (or "null" for each field when data.valid == false).
    bool publishSensorData(const SensorData& data);

    bool publishControllerTemp(float tempC);

    // Typical values: "OK" or "SENSOR FAULT"
    bool publishControllerStatus(const char* status);

private:
    const char* _ssid;
    const char* _wifiPass;
    const char* _broker;
    int         _port;
    const char* _user;
    const char* _mqttPass;
    const char* _clientId;
    PWMManager* _pwmManager;

    WiFiClient _wifiClient;
    MqttClient _mqttClient;

    bool connectWiFi();
    bool connectMQTT();
    bool publish(const char* topic, const char* payload);
    void subscribeToTopics();
    void handleMessage(int messageSize);
};
