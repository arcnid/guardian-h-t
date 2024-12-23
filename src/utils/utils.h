// utils.h
#ifndef UTILS_H
#define UTILS_H

#include <ESP8266WebServer.h>
#include <PubSubClient.h>

// Constants
#define MAX_SSID_LENGTH     32
#define MAX_PASSWORD_LENGTH 64
#define UUID_LENGTH         36
#define DEVICEID_LENGTH     36

// Pin Definitions
#define SHELLY_BUILTIN_LED 0

// Configuration Structure
struct Config {
  char ssid[MAX_SSID_LENGTH];
  char password[MAX_PASSWORD_LENGTH];
  char uuid[UUID_LENGTH];
  char deviceId[DEVICEID_LENGTH];
  uint32_t checksum; // For data integrity
};

// External Variables
extern Config storedConfig;
extern PubSubClient mqttClient;

// MQTT Topics
extern const char* mqtt_publish_topic;
extern const char* mqtt_subscribe_topic;

extern bool doesUserExist;

// Function Prototypes
void sendResponse(ESP8266WebServer &server, int statusCode, const String &content);
void flashLED();
bool connect(const String& ssid, const String& password);
bool saveUserAndWifiCreds(const String& ssid, const String& password, const String& uuid, const String& deviceId);
bool checkForWifiAndUser();
uint32_t calculateChecksum(const uint8_t* data, size_t length);
void clearEEPROM();

// MQTT Function Prototypes
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool connectToMQTT();
void publishMessage(const char* topic, const char* message);

#endif // UTILS_H
