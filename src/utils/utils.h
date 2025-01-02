// utils.h
#ifndef UTILS_H
#define UTILS_H

#include <ESP8266WebServer.h>
#include <PubSubClient.h>
#include <HTS221Sensor.h>
#include <SPI.h>
#include <string.h>   // for memcpy
#include <stdlib.h>

#define MQTT_MAX_PACKET_SIZE 512

// Constants
#define MAX_SSID_LENGTH     32
#define MAX_PASSWORD_LENGTH 64
#define UUID_LENGTH         36
#define DEVICEID_LENGTH     36

// Pin Definitions
#define SHELLY_BUILTIN_LED 0
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

// SPI Pin Definitions
#define PIN_HSPI_CS     15
#define PIN_HSPI_MISO   12
#define PIN_HSPI_MOSI   13
#define PIN_HSPI_SCLK   14

// Shelly H&T low-power MCU reset pin
#define PIN_RESET       5

// SPI Settings
#define HSPI_FREQ       200000     // 200 kHz
#define HSPI_MODE       SPI_MODE3  // CPOL=1, CPHA=1
#define HSPI_BITORDER   MSBFIRST   // MSB first

// Frame & Command Constants
#define FILL_BYTE          0xAA
#define HEADER_BYTE1       0xAA
#define HEADER_BYTE2       0x99
#define FOOTER_BYTE1       0xAA
#define FOOTER_BYTE2       0xAA
#define SLAVE_FOOTER_BYTE2 0x79  // The final byte the Shelly slave might send

// Commands
#define CMD_READ_MEASUREMENTS  0x01
#define CMD_LED                0x02
#define CMD_PREVENT_SLEEP      0x04
#define CMD_UNKNOWN_06         0x06
#define CMD_UNKNOWN_07         0x07

// LED status
#define LED_STATUS_OFF         0x02
#define LED_STATUS_BLINK_SLOW  0x03

// Some debug placeholders
#define UNDEFINED -999.0

// Minimal logging: Only report temperature and humidity
#define REPORT_TEMPHUM(t, h) Serial.printf("Temperature: %.2f°C, Humidity: %.2f%%\n", t, h)

extern TwoWire dev_i2c;

// Structure to hold sensor data
struct SensorData {
    float temperature;
    float humidity;
    bool success;
};

// Function Prototypes
void sendResponse(ESP8266WebServer &server, int statusCode, const String &content);
void flashLED();
bool connectToWiFi(const String& ssid, const String& password);
bool saveUserAndWifiCreds(const String& ssid, const String& password, const String& uuid, const String& deviceId);
bool checkForWifiAndUser();
uint32_t calculateChecksum(const uint8_t* data, size_t length);
void clearEEPROM();

// MQTT Function Prototypes
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool connectToMQTT();
void publishMessage(const char* topic, const char* message);

// Sensor and I²C Function Prototypes
SensorData readSensorData();
void sendHeartbeat();
uint8_t findI2CAddress();
void wakeUpSTM32();
void setHTS221ThresholdsManual();
void scanI2CDevices();
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

// SPI Function Prototypes
void hspiSetup();
void hspiTransfer(const uint8_t* out, uint8_t* in, uint32_t len);
void readShellyHTData();
bool sendCmd_ReadMeasurements();
bool recvMeasurements(double* temperature, double* humidity);
bool sendCmd_LED(uint8_t led_status);
bool sendCmd_PreventSleep();
bool sendCmd_06();
bool sendCmd_07();
uint8_t* makeMOSIFrame(uint8_t cmd, const uint8_t* data, uint8_t data_len, uint8_t* out_len);
uint8_t* makeMOSIFillFrame(uint8_t len);
bool misoFrameValid(const uint8_t* frame, uint8_t frame_len);
void wakeUpSensor();
void resetLowPowerMCU();
void setupSTM();
String getDeviceId();
void sendSensorMessage(float temperature, float humidity);
String getTopic();
String getUserId();


void sensorLoop();
#endif // UTILS_H
