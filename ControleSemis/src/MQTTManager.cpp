#include "MQTTManager.h"
#include <Arduino.h>

#include "MQTTManager.h"
#include "PWMManager.h"
#include <Arduino.h>

// ── MQTT topics ──────────────────────────────────────────────────────────────
static const char* TOPIC_AIR_TEMP    = "agriculture/semis/air/temperature";
static const char* TOPIC_AIR_HUM     = "agriculture/semis/air/humidity";
static const char* TOPIC_AIR_CO2     = "agriculture/semis/air/CO2";
static const char* TOPIC_CTRL_TEMP   = "agriculture/semis/controller/temperature";
static const char* TOPIC_CTRL_STATE  = "agriculture/semis/controller/state";
static const char* TOPIC_CMD_FAN     = "agriculture/semis/cmd/fan";

// ── Construction ─────────────────────────────────────────────────────────────
MQTTManager::MQTTManager(const char* ssid,     const char* wifiPass,
                         const char* broker,   int         port,
                         const char* user,     const char* mqttPass,
                         const char* clientId,
                         PWMManager* pwmManager)
    : _ssid(ssid), _wifiPass(wifiPass),
      _broker(broker), _port(port),
      _user(user), _mqttPass(mqttPass),
      _clientId(clientId),
      _pwmManager(pwmManager),
      _mqttClient(_wifiClient)
{}

// ── Connection ───────────────────────────────────────────────────────────────
bool MQTTManager::connect() {
    if (!connectWiFi())  return false;
    if (!connectMQTT())  return false;
    return true;
}

bool MQTTManager::connectWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        #ifdef DEBUG
        Serial.println("[WiFi] Already connected");
        #endif
        return true;
    }

    #ifdef DEBUG
    Serial.print("[WiFi] Connecting to SSID: ");
    Serial.println(_ssid);
    #endif
    WiFi.begin(_ssid, _wifiPass);

    unsigned long start = millis();
    int attempt = 0;
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > 15000UL) {
            #ifdef DEBUG
            Serial.print("[WiFi] Connection timeout after ");
            Serial.print(millis() - start);
            Serial.println(" ms");
            #endif
            return false;
        }
        if (++attempt % 6 == 0) {  // every 3 seconds
            #ifdef DEBUG
            Serial.print("[WiFi] Connecting... status = ");
            Serial.println(WiFi.status());
            #endif
        }
        delay(500);
    }

    #ifdef DEBUG
    Serial.print("[WiFi] Connected! IP: ");
    Serial.println(WiFi.localIP());
    #endif

    // Ensure we have a valid datapoint before MQTT connect.
    unsigned long ipStart = millis();
    while (WiFi.localIP() == IPAddress(0,0,0,0)) {
        if (millis() - ipStart > 10000UL) {
            #ifdef DEBUG
            Serial.println("[WiFi] Waiting for valid IP timed out");
            #endif
            return false;
        }
        delay(200);
    }

    #ifdef DEBUG
    Serial.print("[WiFi] Final IP: ");
    Serial.println(WiFi.localIP());
    #endif

    return true;
}

bool MQTTManager::connectMQTT() {
    if (_mqttClient.connected()) {
        #ifdef DEBUG
        Serial.println("[MQTT] Already connected");
        #endif
        return true;
    }

    #ifdef DEBUG
    Serial.print("[MQTT] Connecting to broker: ");
    Serial.print(_broker);
    Serial.print(":");
    Serial.println(_port);
    #endif

    _mqttClient.setId(_clientId);
    _mqttClient.setUsernamePassword(_user, _mqttPass);

    const int maxAttempts = 3;
    const unsigned long retryDelayMs = 1000UL;
    bool success = false;

    for (int attempt = 1; attempt <= maxAttempts; attempt++) {
        success = _mqttClient.connect(_broker, _port);
        if (success) {
            #ifdef DEBUG
            Serial.print("[MQTT] Connected successfully on attempt ");
            Serial.println(attempt);
            #endif
            subscribeToTopics();
            return true;
        }

        #ifdef DEBUG
        Serial.print("[MQTT] Connection failed attempt ");
        Serial.print(attempt);
        Serial.print("/"
                     );
        Serial.print(maxAttempts);
        Serial.println(" (check broker address/port/credentials)");
        #endif

        if (attempt < maxAttempts) {
            delay(retryDelayMs);
        }
    }

    #ifdef DEBUG
    Serial.println("[MQTT] All connection attempts failed");
    #endif
    return false;
}

void MQTTManager::subscribeToTopics() {
    #ifdef DEBUG
    Serial.print("[MQTT] Subscribing to: ");
    Serial.println(TOPIC_CMD_FAN);
    #endif

    _mqttClient.subscribe(TOPIC_CMD_FAN);

    #ifdef DEBUG
    Serial.println("[MQTT] Subscription request sent");
    #endif
}

void MQTTManager::handleMessage(int messageSize) {
    String topic = _mqttClient.messageTopic();

    #ifdef DEBUG
    Serial.print("[MQTT:Message] Topic: ");
    Serial.println(topic);
    #endif

    // Read the message payload
    String payload;
    while (_mqttClient.available()) {
        payload += (char)_mqttClient.read();
    }

    #ifdef DEBUG
    Serial.print("[MQTT:Message] Payload: ");
    Serial.println(payload);
    #endif

    // Handle fan command
    if (topic == TOPIC_CMD_FAN) {
        if (_pwmManager != nullptr) {
            int percent = payload.toInt();
            #ifdef DEBUG
            Serial.print("[MQTT:Message] Setting fan PWM to ");
            Serial.print(percent);
            Serial.println("%");
            #endif
            _pwmManager->setDutyCycle(percent);
        } else {
            #ifdef DEBUG
            Serial.println("[MQTT:Message] ⚠ PWMManager not available!");
            #endif
        }
    }
}

bool MQTTManager::isConnected() {
    return WiFi.status() == WL_CONNECTED && _mqttClient.connected();
}

void MQTTManager::poll() {
    int messageSize = _mqttClient.parseMessage();
    if (messageSize > 0) {
        handleMessage(messageSize);
    }
}

// ── Publishing ───────────────────────────────────────────────────────────────
bool MQTTManager::publish(const char* topic, const char* payload) {
    _mqttClient.beginMessage(topic);
    _mqttClient.print(payload);
    bool success = _mqttClient.endMessage();

    #ifdef DEBUG
    Serial.print("[MQTT] publish(");
    Serial.print(topic);
    Serial.print(") = ");
    Serial.print(payload);
    Serial.print(" → ");
    Serial.println(success ? "OK" : "FAIL");
    #endif

    return success;
}

bool MQTTManager::publishSensorData(const SensorData& data) {
    char buf[16];
    bool ok = true;

    if (data.valid) {
        dtostrf(data.temperature, 5, 1, buf);
        ok &= publish(TOPIC_AIR_TEMP, buf);

        dtostrf(data.humidity, 5, 1, buf);
        ok &= publish(TOPIC_AIR_HUM, buf);

        dtostrf(data.co2, 6, 0, buf);
        ok &= publish(TOPIC_AIR_CO2, buf);
    } else {
        ok &= publish(TOPIC_AIR_TEMP, "null");
        ok &= publish(TOPIC_AIR_HUM,  "null");
        ok &= publish(TOPIC_AIR_CO2,  "null");
    }

    return ok;
}

bool MQTTManager::publishControllerTemp(float tempC) {
    char buf[16];
    dtostrf(tempC, 5, 1, buf);
    return publish(TOPIC_CTRL_TEMP, buf);
}

bool MQTTManager::publishControllerStatus(const char* status) {
    return publish(TOPIC_CTRL_STATE, status);
}
