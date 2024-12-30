#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <time.h>

// Include your custom headers
#include "utils/utils.h"
#include "api/api.h"

// Web server on port 80
ESP8266WebServer server(80);

// Timer variables
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 30000;

unsigned long lastReconnectAttempt = 5000;
const unsigned long reconnectInterval = 10000;

unsigned long lastWifiRetryAttempt = 0;
const unsigned long wifiRetryInterval = 30000;

unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatInterval = 60000;

// I²C Keep-Alive Timer
unsigned long lastI2CKeepAliveTime = 0;
const unsigned long i2cKeepAliveInterval = 120000; // 2 minutes

// GPIO Constants
const uint8_t targetI2CAddress = 0x01;
const int sdaPin = 5;  // SDA: GPIO5
const int sclPin = 4;  // SCL: GPIO4

void startAccessPoint() {
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP("Guardian_GMS_Sensor", "");
    Serial.print("Access point IP: ");
    Serial.println(WiFi.softAPIP());
    digitalWrite(SHELLY_BUILTIN_LED, LOW);
}

void synchronizeTime() {
    Serial.println("Synchronizing system time...");
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");

    struct tm timeinfo;
    while (!getLocalTime(&timeinfo)) {
        Serial.println("Waiting for time synchronization...");
        delay(1000);
    }

    char timeStr[64];
    strftime(timeStr, sizeof(timeStr), "Current time: %A, %B %d %Y %H:%M:%S", &timeinfo);
    Serial.println(timeStr);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nFirmware Started");

    EEPROM.begin(512);

    pinMode(SHELLY_BUILTIN_LED, OUTPUT);
    digitalWrite(SHELLY_BUILTIN_LED, HIGH);

    if (checkForWifiAndUser()) {
        if (connect(String(storedConfig.ssid), String(storedConfig.password))) {
            Serial.println("Connected to WiFi successfully.");
            digitalWrite(SHELLY_BUILTIN_LED, HIGH);
        } else {
            Serial.println("WiFi connection failed, starting Access Point...");
            startAccessPoint();
        }
    } else {
        Serial.println("No Credentials Found, Starting Access Point...");
        startAccessPoint();
    }

    if (WiFi.status() == WL_CONNECTED) {
        synchronizeTime();
    } else {
        Serial.println("WiFi not connected. Unable to synchronize time.");
    }

    setupApiRoutes(server);
    server.begin();
    Serial.println("Web Server Started");

    if (doesUserExist && WiFi.status() == WL_CONNECTED) {
        if (connectToMQTT()) {
            Serial.println("MQTT Connected Successfully.");
        } else {
            Serial.println("Failed to Connect to MQTT Broker.");
        }
    }

    initializeSensor();

    // Initialize I²C
    Wire.begin(sdaPin, sclPin);
    Serial.println("I²C Initialized for Keep-Alive.");
}

void keepAliveI2C() {
    Serial.println("🔄 [I²C Keep-Alive] Sending periodic I²C ping...");
    Wire.beginTransmission(targetI2CAddress);
    Wire.write(0x00); // Attempt to write to a register
    if (Wire.endTransmission() == 0) {
        Serial.println("✅ [I²C Keep-Alive] Communication succeeded, device awake.");
    } else {
        Serial.println("❌ [I²C Keep-Alive] Communication failed.");
    }
}

void loop() {
    server.handleClient();
    unsigned long currentMillis = millis();

    // WiFi Reconnect
    if (doesUserExist && (WiFi.status() != WL_CONNECTED)) {
        if (currentMillis - lastWifiRetryAttempt > wifiRetryInterval) {
            lastWifiRetryAttempt = currentMillis;
            Serial.println("Attempting to reconnect to WiFi...");
            if (connect(String(storedConfig.ssid), String(storedConfig.password))) {
                Serial.println("Reconnected to WiFi successfully.");
                digitalWrite(SHELLY_BUILTIN_LED, HIGH);
                synchronizeTime();

                if (!mqttClient.connected()) {
                    if (connectToMQTT()) {
                        Serial.println("MQTT Connected Successfully after WiFi reconnect.");
                    } else {
                        Serial.println("Failed to Connect to MQTT Broker after WiFi reconnect.");
                    }
                }
            } else {
                Serial.println("WiFi reconnect attempt failed.");
            }
        }
    }

    // MQTT Reconnect
    if (doesUserExist && (WiFi.status() == WL_CONNECTED)) {
        if (!mqttClient.connected()) {
            if (currentMillis - lastReconnectAttempt > reconnectInterval) {
                lastReconnectAttempt = currentMillis;
                Serial.println("Reconnecting to MQTT Broker...");
                if (connectToMQTT()) {
                    Serial.println("Reconnected to MQTT Broker.");
                } else {
                    Serial.println("Failed to connect to broker");
                }
            }
        }
        mqttClient.loop();

        if (currentMillis - lastPublishTime >= publishInterval) {
            lastPublishTime = currentMillis;
            String status = "Device is online. IP: " + WiFi.localIP().toString();
            publishMessage(mqtt_publish_topic, status.c_str());
        }
    }

    // Heartbeat Signal
    if (currentMillis - lastHeartbeatTime >= heartbeatInterval) {
        lastHeartbeatTime = currentMillis;
        sendHeartbeat();
    }

    // I²C Keep-Alive Every 2 Minutes
    if (currentMillis - lastI2CKeepAliveTime >= i2cKeepAliveInterval) {
        lastI2CKeepAliveTime = currentMillis;
        keepAliveI2C();
        scanI2CBus();
    }
}
