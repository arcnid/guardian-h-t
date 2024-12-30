// utils.h
#ifndef UTILS_H
#define UTILS_H

#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <HTS221Sensor.h>

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

#define I2C_SDA    4  // GPIO4 (D2)
#define I2C_SCL    5  // GPIO5 (D1)

#define UART_MIN 0x00
#define UART_MAX 0xFF

#define ALT_SDA_PIN 12 // Alternate SDA
#define ALT_SCL_PIN 14 // Alternate SCL

extern TwoWire dev_i2c;


struct SensorData {
    float temperature;
    float humidity;
    bool success;
};

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
SensorData readSensorData();
void initializeSensor();
void sendHeartbeat();
uint8_t findI2CAddress();
void wakeUpSTM32();
void setHTS221ThresholdsManual();
void scanI2CDevices();
void dumpI2CRegisters(uint8_t deviceAddress);
void scanI2C(uint8_t sda, uint8_t scl);
void readHTS221Raw();
void testI2CBusHealth();
void forceHTS221Init();
void runFullDiagnostics();
void testAllGPIOPins();
void readAllGPIOPins();
void pulseLowGPIOPins();
void advancedButtonSequence();
void gpio5WithWakeUp();
void bruteForceI2CRegisters(uint8_t deviceAddress);

#endif // UTILS_H
