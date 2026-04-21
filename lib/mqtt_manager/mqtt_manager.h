#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

class MqttManager
{
public:
    MqttManager(const char *wifiSsid,
                const char *wifiPass,
                const char *mqttServer,
                uint16_t mqttPort = 1883,
                const char *mqttUser = nullptr,
                const char *mqttPass = nullptr);

    void begin();
    void loop();
    bool isConnected() const;
    bool publish(const char *topic, const String &payload);

private:
    const char *ssid;
    const char *password;
    const char *server;
    const char *user;
    const char *pass;
    uint16_t port;
 
    WiFiClient wifiClient;
    PubSubClient mqttClient;

    unsigned long lastReconnectAttempt{0};
    static constexpr unsigned long RECONNECT_INTERVAL = 5000;

    void connectWiFi();
    bool connectMQTT();
};
