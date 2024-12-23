// utils.cpp
#include "utils.h"
#include "secrets.h" // Include secrets.h for MQTT credentials
#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>

// Global Variables
Config storedConfig;

// For testing with a public broker that doesn't require TLS:
// Example: HiveMQ public broker
// MQTT_SERVER = "broker.hivemq.com"
// MQTT_PORT = 1883
const char* mqtt_publish_topic    = "/gms/user/5d62be50-ec8f-48f4-b576-a240e42db065";
const char* mqtt_subscribe_topic  = "/gms/user/5d62be50-ec8f-48f4-b576-a240e42db065";

// Initialize MQTT Client with a plain WiFiClient
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Define the global state variable
bool doesUserExist = false;

// Utility Functions

// Calculate Checksum for Data Integrity
uint32_t calculateChecksum(const uint8_t* data, size_t length) {
  uint32_t checksum = 0;
  for (size_t i = 0; i < length; ++i) {
    checksum += data[i];
  }
  return checksum;
}

// Save User and WiFi Credentials to EEPROM
bool saveUserAndWifiCreds(const String& ssid, const String& password, const String& uuid, const String& deviceId) {
  // Clear the Config struct
  memset(&storedConfig, 0, sizeof(Config));

  // Copy SSID, password, UUID, and DeviceID into the struct
  ssid.toCharArray(storedConfig.ssid, MAX_SSID_LENGTH);
  password.toCharArray(storedConfig.password, MAX_PASSWORD_LENGTH);
  uuid.toCharArray(storedConfig.uuid, UUID_LENGTH);
  deviceId.toCharArray(storedConfig.deviceId, DEVICEID_LENGTH);

  // Calculate checksum excluding the checksum field itself
  storedConfig.checksum = 0;
  storedConfig.checksum = calculateChecksum(reinterpret_cast<uint8_t*>(&storedConfig), sizeof(Config) - sizeof(uint32_t));

  // Write the Config struct to EEPROM
  for (size_t i = 0; i < sizeof(Config); ++i) {
    EEPROM.write(i, *((uint8_t*)&storedConfig + i));
  }

  // Commit changes to EEPROM
  if (EEPROM.commit()) {
    Serial.println("Configuration saved to EEPROM successfully.");
    return true;
  } else {
    Serial.println("Failed to commit EEPROM changes.");
    return false;
  }
}

// Check for WiFi and User Configuration in EEPROM
bool checkForWifiAndUser() {
  // Read the Config struct from EEPROM
  for (size_t i = 0; i < sizeof(Config); ++i) {
    *((uint8_t*)&storedConfig + i) = EEPROM.read(i);
  }

  // Calculate checksum of the read data
  uint32_t calculatedChecksum = calculateChecksum(reinterpret_cast<uint8_t*>(&storedConfig), sizeof(Config) - sizeof(uint32_t));

  // Verify checksum
  if (storedConfig.checksum == calculatedChecksum) {
    // Check if all required fields are non-empty
    if (strlen(storedConfig.ssid) == 0 || strlen(storedConfig.password) == 0 ||
        strlen(storedConfig.uuid) == 0 || strlen(storedConfig.deviceId) == 0) {
      Serial.println("Configuration found but one or more fields are empty.");
      doesUserExist = false;
      return false;
    }

    Serial.println("Valid configuration found in EEPROM.");
    Serial.printf("SSID: %s\n", storedConfig.ssid);
    Serial.printf("UUID: %s\n", storedConfig.uuid);
    Serial.printf("DeviceID: %s\n", storedConfig.deviceId);

    doesUserExist = true;
    return true;
  } else {
    Serial.println("Invalid or no configuration found in EEPROM.");
    doesUserExist = false;
    return false;
  }
}

// Send HTTP Response with CORS Headers
void sendResponse(ESP8266WebServer &server, int statusCode, const String &content) {
  // Debug output
  Serial.printf("[DEBUG] Sending response with status code %d, content: %s\n", statusCode, content.c_str());

  // Set headers for CORS
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");

  // Send the response
  server.send(statusCode, "application/json", content);
}

// Flash LED (Example Function)
void flashLED() {
  digitalWrite(SHELLY_BUILTIN_LED, LOW);  // Turn LED on
  delay(500);
  digitalWrite(SHELLY_BUILTIN_LED, HIGH); // Turn LED off
  delay(500);
}

// Connect to WiFi with Given Credentials
bool connect(const String& ssid, const String& password) {
  Serial.println("Attempting to connect to WiFi...");

  // Turn on the LED (solid, indicating connection attempt)
  digitalWrite(SHELLY_BUILTIN_LED, LOW);

  // Disconnect any previous connection
  WiFi.disconnect();
  delay(100);

  // Start WiFi connection
  WiFi.begin(ssid.c_str(), password.c_str());

  // Wait until connected or timeout (15 seconds)
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 15000) {
    delay(500);
    Serial.print(".");
    Serial.printf(" Current WiFi.status(): %d\n", WiFi.status());
  }

  // Check connection status
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.printf("Signal Strength: %d dBm\n", WiFi.RSSI());
    digitalWrite(SHELLY_BUILTIN_LED, HIGH);

    doesUserExist = true;
    return true;
  } else {
    Serial.println("\nFailed to connect to WiFi.");
    Serial.printf("WiFi.status(): %d\n", WiFi.status());

    doesUserExist = false;
    return false;
  }
}

// Clear EEPROM (Use with Caution)
void clearEEPROM() {
  EEPROM.begin(512);
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0xFF);
  }
  EEPROM.commit();
  Serial.println("EEPROM cleared.");
}

// MQTT Callback Function to Handle Incoming Messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);

  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Message: ");
  Serial.println(message);

  // Handle the message (example: toggle LED)
  if (String(topic) == mqtt_subscribe_topic) {
    if (message == "ON") {
      digitalWrite(SHELLY_BUILTIN_LED, LOW);
      Serial.println("LED turned ON via MQTT");
    } else if (message == "OFF") {
      digitalWrite(SHELLY_BUILTIN_LED, HIGH);
      Serial.println("LED turned OFF via MQTT");
    }
  }
}

// Connect to MQTT Broker
bool connectToMQTT() {
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  String clientId = "ESP8266Client-" + String(WiFi.macAddress()) + "-" + String(storedConfig.deviceId);
  Serial.println(clientId);
  Serial.printf("Connecting to MQTT Broker at %s:%d...\n", MQTT_SERVER, MQTT_PORT);

  bool connected = mqttClient.connect(clientId.c_str());
  
  if (connected) {
    Serial.println("Connected to MQTT Broker.");
    mqttClient.subscribe(mqtt_subscribe_topic);
    Serial.print("Subscribed to topic: ");
    Serial.println(mqtt_subscribe_topic);
    mqttClient.publish(mqtt_publish_topic, "ESP8266 Connected");
    return true;
  } else {
    Serial.print("Failed to connect to MQTT Broker, state: ");
    Serial.println(mqttClient.state());
    return false;
  }
}

// Publish Message to MQTT Broker
void publishMessage(const char* topic, const char* message) {
  if (mqttClient.publish(topic, message)) {
    Serial.print("Message published to topic ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.print("Failed to publish message to topic ");
    Serial.println(topic);
  }
}
