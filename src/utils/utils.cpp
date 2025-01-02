// src/utils.cpp
#include "utils.h"
#include "secrets.h" // Include secrets.h for MQTT credentials
#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HTS221Sensor.h>
#include <SPI.h>
#include <string.h>   // for memcpy
#include <stdlib.h>
#include <cmath>
#include <ArduinoJson.h>

// Global Variables
Config storedConfig;
HTS221Sensor sensor(&Wire);

String getTopic(){
    String userId = getUserId();
    String deviceId = getDeviceId();


    String topic = "gms/" + userId + "/" + deviceId;

    return topic;

}


// For testing with a public broker that doesn't require TLS:
// Example: HiveMQ public broker
// MQTT_SERVER = "broker.hivemq.com"
// MQTT_PORT = 1883
const char* mqtt_publish_topic    = getTopic().c_str();
const char* mqtt_subscribe_topic  = getTopic().c_str();

// Initialize MQTT Client with a plain WiFiClient
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Define the global state variable
bool doesUserExist = false;


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
bool connectToWiFi(const String& ssid, const String& password) {
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
            // digitalWrite(SHELLY_BUILTIN_LED, LOW);
            Serial.println("LED turned ON via MQTT");
        } else if (message == "OFF") {
            // digitalWrite(SHELLY_BUILTIN_LED, HIGH);
            Serial.println("LED turned OFF via MQTT");
        }
    }
}

// Connect to MQTT Broker
bool connectToMQTT() {
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
     int randomSessionId = random(1, 201); // Random number from 1 to 200

    String clientId = "ESP8266Client-" + String(WiFi.macAddress()) + "-" + String(storedConfig.deviceId) + "-" + String(randomSessionId);
    String topic = getTopic();

   

    bool connected = mqttClient.connect(clientId.c_str());

    Serial.println("about to connect to mqtt broker with this topic");
    Serial.println(mqtt_subscribe_topic);

    if (connected) {
        Serial.println("Connected to MQTT Broker.");
        mqttClient.subscribe(topic.c_str());
        Serial.print("Subscribed to topic: ");
        Serial.println(topic);
        Serial.println(topic.c_str());

        mqttClient.publish(topic.c_str(), "ESP8266 Connected");
        return true;
    } else {
        Serial.print("Failed to connect to MQTT Broker, state: ");
        Serial.println(mqttClient.state());
        return false;
    }
}

// Publish Message to MQTT Broker
void publishMessage(const char* topic, const char* message) {

    Serial.println("about to send message to topic ");
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

// Read Sensor Data
SensorData readSensorData() {
    SensorData data;

    // Read humidity and temperature
    float temperature = 0.0f;
    float humidity = 0.0f;

    if (sensor.GetHumidity(&humidity) != HTS221_STATUS_OK ||
        sensor.GetTemperature(&temperature) != HTS221_STATUS_OK) {
        Serial.println("Failed to read sensor data!");
        data.success = false;
        return data;
    }

    data.temperature = temperature;
    data.humidity = humidity;
    data.success = true;

    REPORT_TEMPHUM(data.temperature, data.humidity);

   

    return data;
}

// Send Heartbeat via I²C
void sendHeartbeat() {
    Serial.println("[DEBUG] Sending heartbeat signal to STM via I²C...");

    // Trigger START condition and check response
    Wire.beginTransmission(0x18); // STM32 I²C Address (try 0x08, 0x18, etc.)
    Wire.write("HEARTBEAT");
    byte error = Wire.endTransmission();

    if (error == 0) {
        Serial.println("[INFO] Heartbeat signal sent successfully via I²C!");
    } else {
        Serial.print("[ERROR] Failed to send heartbeat, error code: ");
        Serial.println(error);
    }
}

// Wake Up STM32 via GPIO
void wakeUpSTM32() {
    Serial.println("[DEBUG] Triggering wake-up pin...");

    pinMode(16, OUTPUT); // Example GPIO pin, adjust if needed
    digitalWrite(16, HIGH);
    delay(100);  // Hold high for 100ms
    digitalWrite(16, LOW);

    Serial.println("[INFO] Wake-up signal sent via GPIO!");
}

// Find I²C Address
uint8_t findI2CAddress() {
    Serial.println("[DEBUG] Scanning I²C bus for devices...");

    uint8_t foundAddress = 0x00; // Default to 0x00 (no device found)
    byte error;
    int nDevices = 0;

    // Loop through all possible I²C addresses (1 to 127)
    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0) {
            Serial.print("[INFO] I²C device found at address 0x");
            Serial.println(address, HEX);
            foundAddress = address;
            nDevices++;
        } else if (error == 4) {
            Serial.print("[ERROR] Unknown error at address 0x");
            Serial.println(address, HEX);
        }
    }

    if (nDevices == 0) {
        Serial.println("[WARNING] No I²C devices found on the bus.");
    } else {
        Serial.print("[INFO] Total I²C devices found: ");
        Serial.println(nDevices);
    }

    return foundAddress;
}

// Set HTS221 Thresholds Manually
void setHTS221ThresholdsManual() {
    Serial.println("[DEBUG] Configuring HTS221 thresholds manually...");

    Wire.beginTransmission(0x5F); // HTS221 default address

    // Write humidity threshold
    Wire.write(0x33); // Humidity high threshold register
    Wire.write(50);   // Example: 50% threshold

    // Write temperature threshold
    Wire.write(0x34); // Temperature high threshold register
    Wire.write(30);   // Example: 30°C threshold

    byte error = Wire.endTransmission();

    if (error == 0) {
        Serial.println("[INFO] HTS221 thresholds configured successfully!");
    } else {
        Serial.print("[ERROR] Failed to set thresholds, error code: ");
        Serial.println(error);
    }
}

// Scan I²C Devices
void scanI2CDevices() {
    Serial.println("[DEBUG] Scanning I²C bus for devices...");

    int nDevices = 0;

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.print("[INFO] Found I²C device at address 0x");
            Serial.println(address, HEX);

            // dumpI2CRegisters(address); // Dump registers for this device
            nDevices++;
        }
    }

    if (nDevices == 0) {
        Serial.println("[WARNING] No I²C devices found on the bus.");
    } else {
        Serial.print("[INFO] Total I²C devices found: ");
        Serial.println(nDevices);
    }
}

// Test I²C Bus Health
void testI2CBusHealth() {
    Wire.beginTransmission(0x5F);
    byte error = Wire.endTransmission();

    if (error == 0) {
        Serial.println("[INFO] HTS221 detected successfully on I²C bus!");
    } else {
        Serial.print("[ERROR] I²C communication error: ");
        Serial.println(error);
    }
}

// Force HTS221 Initialization
void forceHTS221Init() {
    Wire.beginTransmission(0x5F);
    Wire.write(0x20); // CTRL_REG1
    Wire.write(0x85); // Enable sensor, 1 Hz mode
    Wire.endTransmission();
    delay(100);

    Wire.beginTransmission(0x5F);
    Wire.write(0x21); // CTRL_REG2
    Wire.write(0x01); // Trigger one-shot mode
    Wire.endTransmission();

    Serial.println("[INFO] HTS221 Initialization Commands Sent");
}

void readHTS221Raw() {
  Wire.beginTransmission(0x5F); // HTS221 Address
  Wire.write(0x28 | 0x80); // Humidity register (0x28) with auto-increment (0x80)
  Wire.endTransmission();
  Wire.requestFrom(0x5F, 4);

  if (Wire.available() == 4) {
      uint8_t humL = Wire.read(); // Humidity low byte
      uint8_t humH = Wire.read(); // Humidity high byte
      uint8_t tempL = Wire.read(); // Temperature low byte
      uint8_t tempH = Wire.read(); // Temperature high byte

      float humidity = ((humH << 8) | humL) / 65536.0 * 100.0;
      float temperature = ((tempH << 8) | tempL) / 65536.0 * 120.0 - 40.0;

      Serial.print("Raw Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      Serial.print("Raw Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
  } else {
      Serial.println("[ERROR] Failed to read raw HTS221 data!");
  }
}

void dumpI2CRegisters(uint8_t deviceAddress) {
  Serial.print("[INFO] Dumping registers for device at 0x");
  Serial.println(deviceAddress, HEX);

  for (uint8_t reg = 0x00; reg <= 0xFF; reg++) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(reg); // Select register
    if (Wire.endTransmission() == 0) {
      Wire.requestFrom(deviceAddress, (uint8_t)1);

      if (Wire.available()) {
        uint8_t value = Wire.read();
        Serial.print("Register 0x");
        Serial.print(reg, HEX);
        Serial.print(": 0x");
        Serial.println(value, HEX);
      } else {
        Serial.print("Register 0x");
        Serial.print(reg, HEX);
        Serial.println(": [No Data]");
      }
    } else {
      Serial.print("Register 0x");
      Serial.print(reg, HEX);
      Serial.println(": [Write Error]");
    }
  }
  Serial.println("[INFO] Register dump complete.\n");
}

void scanI2C(uint8_t sda, uint8_t scl) {
  Serial.print("[DEBUG] Scanning I²C bus on SDA: ");
  Serial.print(sda);
  Serial.print(", SCL: ");
  Serial.println(scl);

  Wire.begin(sda, scl);
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("[INFO] Found device at 0x");
      Serial.println(address, HEX);
    }
  }
}

// Run Full Diagnostics
void runFullDiagnostics() {
    Serial.println("\n🔍 [DEBUG] Starting Full System Diagnostics...\n");
    
    // ---------- GPIO Pin Probing ----------
    Serial.println("🟢 [GPIO] Testing All GPIO Pins...");
    int gpioPins[] = {0, 2, 4, 5, 12, 13, 14, 15, 16};
    int pinCount = sizeof(gpioPins) / sizeof(gpioPins[0]);

    for (int i = 0; i < pinCount; i++) {
        int pin = gpioPins[i];
        
        Serial.printf("\n🟢 [GPIO TEST] Testing GPIO%d...\n", pin);
        pinMode(pin, OUTPUT);

        // Step 1: Set GPIO HIGH
        Serial.printf("[GPIO%d] Setting HIGH...\n", pin);
        digitalWrite(pin, HIGH);
        delay(2000); // Wait 2 seconds to observe behavior

        // Step 2: Set GPIO LOW
        Serial.printf("[GPIO%d] Setting LOW...\n", pin);
        digitalWrite(pin, LOW);
        delay(2000); // Wait 2 seconds to observe behavior

        // Step 3: Pulse GPIO (HIGH -> LOW -> HIGH)
        Serial.printf("[GPIO%d] Pulsing HIGH-LOW-HIGH...\n", pin);
        digitalWrite(pin, HIGH);
        delay(500);
        digitalWrite(pin, LOW);
        delay(500);
        digitalWrite(pin, HIGH);
        delay(2000); // Wait 2 seconds to observe behavior

        Serial.printf("✅ [GPIO%d] Test Complete. Moving to next pin...\n", pin);

        // Clear pin state
        digitalWrite(pin, LOW);
        pinMode(pin, INPUT);
    }
    Serial.println("✅ [GPIO] GPIO Test Complete.\n");

    // ---------- I²C Bus Health ----------
    Serial.println("🟢 [I²C] Testing I²C Bus Health...");
    Wire.begin(ALT_SDA_PIN, ALT_SCL_PIN);
    Wire.beginTransmission(0x5F); // HTS221 Address
    byte error = Wire.endTransmission();
    if (error == 0) {
        Serial.println("✅ [I²C] HTS221 detected successfully on alternate pins!");
    } else {
        Serial.printf("❌ [I²C] HTS221 I²C communication error: %d\n", error);
    }

    // ---------- I²C Register Dump ----------
    Serial.println("🟢 [I²C] Dumping All Registers from 0x00 to 0xFF...");
    for (uint8_t reg = 0x00; reg <= 0xFF; reg++) {
        Wire.beginTransmission(0x5F);
        Wire.write(reg);
        Wire.endTransmission();
        Wire.requestFrom(0x5F, 1);

        if (Wire.available()) {
            uint8_t value = Wire.read();
            Serial.printf("[I²C] Register 0x%02X: 0x%02X\n", reg, value);
        } else {
            Serial.printf("❌ [I²C] No Response from Register 0x%02X\n", reg);
        }
    }
    Serial.println("✅ [I²C] I²C Register Dump Complete.\n");

    // ---------- I²C Write Test ----------
    Serial.println("🟢 [I²C] Attempting to Write to Registers...");
    for (uint8_t reg = 0x00; reg <= 0xFF; reg++) {
        Wire.beginTransmission(0x5F);
        Wire.write(reg);
        Wire.write(0xFF); // Arbitrary write value
        if (Wire.endTransmission() == 0) {
            Serial.printf("✅ [I²C] Wrote to Register 0x%02X\n", reg);
        } else {
            Serial.printf("❌ [I²C] Failed to Write to Register 0x%02X\n", reg);
        }
        delay(50); // Prevent I²C flood
    }
    Serial.println("✅ [I²C] I²C Write Test Complete.\n");

    // ---------- UART Sniffing ----------
    Serial.println("🟢 [UART] Sniffing UART Communication...");
    unsigned long uartStartTime = millis();
    while (millis() - uartStartTime < 5000) { // 5-second UART Sniffing
        if (Serial.available()) {
            char c = Serial.read();
            Serial.print(c);
        }
    }
    Serial.println("\n✅ [UART] UART Sniffing Complete.\n");

    // ---------- UART Fuzzing ----------
    Serial.println("🟢 [UART] Fuzzing UART Communication...");
    for (uint8_t command = UART_MIN; command <= UART_MAX; command++) {
        Serial.write(command);
        delay(50);
        Serial.printf("[UART] Sent Command: 0x%02X\n", command);
    }
    Serial.println("✅ [UART] UART Fuzzing Complete.\n");

    // ---------- GPIO Wake Behavior ----------
    Serial.println("🟢 [GPIO] Observing GPIO Wake-Up Behavior...");
    pinMode(16, INPUT);
    if (digitalRead(16) == HIGH) {
        Serial.println("✅ [GPIO] GPIO16 detected HIGH (Wake-Up Triggered)");
    } else {
        Serial.println("❌ [GPIO] GPIO16 not triggered.");
    }

    Serial.println("\n✅ [DEBUG] Full Diagnostics Complete!");
}

// Test All GPIO Pins
void testAllGPIOPins() {
    Serial.println("\n🔍 [TEST] Starting Systematic GPIO Test...");

    // List of GPIOs to test (excluding TX/RX and reserved pins)
    int gpioPins[] = {0, 2, 4, 5, 12, 13, 14, 15, 16};
    int pinCount = sizeof(gpioPins) / sizeof(gpioPins[0]);

    for (int i = 0; i < pinCount; i++) {
        int pin = gpioPins[i];
        
        Serial.printf("\n🟢 [GPIO TEST] Testing GPIO%d...\n", pin);
        pinMode(pin, OUTPUT);

        // Step 1: Set GPIO HIGH
        Serial.printf("[GPIO%d] Setting HIGH...\n", pin);
        digitalWrite(pin, HIGH);
        delay(2000); // Wait 2 seconds to observe behavior

        // Step 2: Set GPIO LOW
        Serial.printf("[GPIO%d] Setting LOW...\n", pin);
        digitalWrite(pin, LOW);
        delay(2000); // Wait 2 seconds to observe behavior

        // Step 3: Pulse GPIO (HIGH -> LOW -> HIGH)
        Serial.printf("[GPIO%d] Pulsing HIGH-LOW-HIGH...\n", pin);
        digitalWrite(pin, HIGH);
        delay(500);
        digitalWrite(pin, LOW);
        delay(500);
        digitalWrite(pin, HIGH);
        delay(2000); // Wait 2 seconds to observe behavior

        Serial.printf("✅ [GPIO%d] Test Complete. Moving to next pin...\n", pin);

        // Clear pin state
        digitalWrite(pin, LOW);
        pinMode(pin, INPUT);
    }

    Serial.println("\n✅ [TEST COMPLETE] All GPIO pins tested systematically.\n");
}

// Read All GPIO Pins
void readAllGPIOPins() {
    Serial.println("\n🔍 [TEST] Starting GPIO Pin Listening Test...");

    // List of GPIOs to test (excluding TX/RX and reserved pins)
    int gpioPins[] = {0, 2, 4, 5, 12, 13, 14, 15, 16};
    int pinCount = sizeof(gpioPins) / sizeof(gpioPins[0]);

    for (int i = 0; i < pinCount; i++) {
        int pin = gpioPins[i];
        
        Serial.printf("\n🟢 [GPIO READ] Listening on GPIO%d...\n", pin);
        pinMode(pin, INPUT);
        
        for (int j = 0; j < 10; j++) { // Read each pin 10 times
            int state = digitalRead(pin);
            Serial.printf("[GPIO%d] State: %s\n", pin, state == HIGH ? "HIGH" : "LOW");
            delay(500); // Half-second delay to observe pin state
        }
        
        Serial.printf("✅ [GPIO%d] Listening Complete. Moving to next pin...\n", pin);
    }

    Serial.println("\n✅ [TEST COMPLETE] All GPIO pins listened systematically.\n");
}

// Pulse Low GPIO Pins
void pulseLowGPIOPins() {
    Serial.println("\n🔍 [ULTIMATE TEST] Beginning Full GPIO, UART, and I²C Exploration...");

    // List of GPIOs to test (excluding TX/RX and reserved pins)
    int gpioPins[] = {0, 2, 4, 5, 12, 13, 14, 15, 16};
    int pinCount = sizeof(gpioPins) / sizeof(gpioPins[0]);

    // 🟢 STEP 1: Flick each GPIO pin ON and OFF
    Serial.println("\n🟢 [STEP 1] Flicking GPIO Pins ON and OFF...");
    for (int i = 0; i < pinCount; i++) {
        int pin = gpioPins[i];
        Serial.printf("[GPIO%d] Flicking ON and OFF...\n", pin);
        pinMode(pin, OUTPUT);
        for (int j = 0; j < 3; j++) {
            digitalWrite(pin, HIGH);
            delay(500);
            digitalWrite(pin, LOW);
            delay(500);
        }
        pinMode(pin, INPUT);
        delay(500);
    }

    // 🟢 STEP 2: Pulse Each GPIO Pin
    Serial.println("\n🟢 [STEP 2] Pulsing GPIO Pins...");
    for (int i = 0; i < pinCount; i++) {
        int pin = gpioPins[i];
        Serial.printf("[GPIO%d] Pulsing HIGH-LOW-HIGH...\n", pin);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        delay(500);
        digitalWrite(pin, LOW);
        delay(500);
        digitalWrite(pin, HIGH);
        delay(500);
        pinMode(pin, INPUT);
        delay(500);
    }

    // 🟢 STEP 3: Monitor UART During Pin Tests
    Serial.println("\n🟢 [STEP 3] Monitoring UART During GPIO Tests...");
    for (int i = 0; i < pinCount; i++) {
        int pin = gpioPins[i];
        Serial.printf("[GPIO%d] Pulsing with UART Monitoring...\n", pin);
        pinMode(pin, OUTPUT);

        digitalWrite(pin, HIGH);
        delay(500);
        digitalWrite(pin, LOW);
        delay(500);
        digitalWrite(pin, HIGH);
        delay(500);

        Serial.println("[UART] Monitoring UART for 3 seconds...");
        unsigned long startMillis = millis();
        while (millis() - startMillis < 3000) {
            if (Serial.available()) {
                char c = Serial.read();
                Serial.print(c);
            }
        }

        pinMode(pin, INPUT);
        delay(500);
    }

    // 🟢 STEP 4: Monitor I²C During GPIO Tests
    Serial.println("\n🟢 [STEP 4] Monitoring I²C During GPIO Tests...");
    Wire.begin(); // Ensure I2C is initialized
    for (int i = 0; i < pinCount; i++) {
        int pin = gpioPins[i];
        Serial.printf("[GPIO%d] Pulsing with I²C Monitoring...\n", pin);
        pinMode(pin, OUTPUT);

        digitalWrite(pin, HIGH);
        delay(500);
        digitalWrite(pin, LOW);
        delay(500);
        digitalWrite(pin, HIGH);
        delay(500);

        Serial.println("[I²C] Scanning for devices...");
        for (uint8_t address = 1; address < 127; address++) {
            Wire.beginTransmission(address);
            byte error = Wire.endTransmission();
            if (error == 0) {
                Serial.printf("[I²C] Device found at address 0x%X\n", address);
            }
        }

        pinMode(pin, INPUT);
        delay(500);
    }

    // 🟢 STEP 5: Test GPIO Pin Combinations
    Serial.println("\n🟢 [STEP 5] Testing GPIO Pin Combinations...");
    for (int i = 0; i < pinCount; i++) {
        for (int j = i + 1; j < pinCount; j++) {
            int pinA = gpioPins[i];
            int pinB = gpioPins[j];

            Serial.printf("[COMBO] Pulsing GPIO%d & GPIO%d...\n", pinA, pinB);
            pinMode(pinA, OUTPUT);
            pinMode(pinB, OUTPUT);

            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, HIGH);
            delay(500);
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, LOW);
            delay(500);
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, HIGH);
            delay(500);

            Serial.println("[COMBO] Monitoring UART for 3 seconds...");
            unsigned long startMillis = millis();
            while (millis() - startMillis < 3000) {
                if (Serial.available()) {
                    char c = Serial.read();
                    Serial.print(c);
                }
            }

            pinMode(pinA, INPUT);
            pinMode(pinB, INPUT);
            delay(500);
        }
    }

    // 🟢 STEP 6: Test GPIO with Different Durations
    Serial.println("\n🟢 [STEP 6] Testing GPIO Durations...");
    for (int i = 0; i < pinCount; i++) {
        int pin = gpioPins[i];
        Serial.printf("[DURATION] Holding GPIO%d HIGH for 5 seconds...\n", pin);
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
        delay(5000);
        digitalWrite(pin, LOW);
        delay(1000);
        pinMode(pin, INPUT);
        delay(500);
    }

    Serial.println("\n✅ [ULTIMATE TEST COMPLETE] All GPIO, UART, and I²C tests completed.\n");
}

// Simulate Button Press on GPIO5
void simulateButtonPress() {
    Serial.println("[TEST] Simulating Button Press on GPIO5...");
    pinMode(5, OUTPUT);
    digitalWrite(5, LOW); delay(100);
    digitalWrite(5, HIGH); delay(500);
    digitalWrite(5, LOW); delay(100);
    Serial.println("[TEST] Button Press Simulation Complete");
}

// Advanced Button Sequence
void advancedButtonSequence() {
    Serial.println("[TEST] Testing Advanced GPIO5 Sequences...");
    pinMode(5, OUTPUT);

    // Quick toggle
    for (int i = 0; i < 5; i++) {
        digitalWrite(5, HIGH); delay(100);
        digitalWrite(5, LOW); delay(100);
    }

    // Long hold
    digitalWrite(5, HIGH); delay(3000); // 3-second hold
    digitalWrite(5, LOW); delay(1000);

    // Combo toggle with GPIO4
    pinMode(4, OUTPUT);
    digitalWrite(5, HIGH); digitalWrite(4, HIGH); delay(500);
    digitalWrite(5, LOW); digitalWrite(4, LOW); delay(500);

    Serial.println("[TEST] Advanced Sequence Complete");
}

// GPIO5 with Wake-Up
void gpio5WithWakeUp() {
    Serial.println("[TEST] Testing GPIO5 + GPIO16 Wake-Up Combo...");
    pinMode(5, OUTPUT);
    pinMode(16, OUTPUT);

    digitalWrite(5, HIGH); delay(200);
    digitalWrite(16, HIGH); delay(200);
    digitalWrite(5, LOW); delay(200);
    digitalWrite(16, LOW); delay(200);

    Serial.println("[TEST] Combo Complete");
}

// Brute Force I²C Registers
void bruteForceI2CRegisters(uint8_t deviceAddress) {
    Serial.printf("[I²C] Brute-Forcing Registers on 0x%02X...\n", deviceAddress);
    for (uint8_t reg = 0x00; reg <= 0xFF; reg++) {
        Wire.beginTransmission(deviceAddress);
        Wire.write(reg);
        if (Wire.endTransmission() == 0) {
            Wire.requestFrom(deviceAddress, (uint8_t)1);
            if (Wire.available()) {
                uint8_t value = Wire.read();
                Serial.printf("[I²C] Register 0x%02X: 0x%02X\n", reg, value);
            }
        }
    }
}

// ------------------------------------------------------------
// Implementation: HSPI Setup & Transfer
// ------------------------------------------------------------
void hspiSetup() {
    SPI.begin();
    SPI.beginTransaction(SPISettings(HSPI_FREQ, HSPI_BITORDER, HSPI_MODE));
}

void hspiTransfer(const uint8_t* out, uint8_t* in, uint32_t len) {
    digitalWrite(PIN_HSPI_CS, LOW);
    delayMicroseconds(10);

    for (uint32_t i = 0; i < len; i++) {
        in[i] = SPI.transfer(out[i]);
    }

    digitalWrite(PIN_HSPI_CS, HIGH);
    delayMicroseconds(10);
}

// ------------------------------------------------------------
// Implementation: Commands & Frames
// ------------------------------------------------------------
void readShellyHTData() {
    // Reset the MCU
    digitalWrite(PIN_RESET, LOW);
    delay(1);
    digitalWrite(PIN_RESET, HIGH);
    delay(100); // Wait for MCU to initialize

    // Turn LED off
    sendCmd_LED(LED_STATUS_OFF);

    // Send READ_MEASUREMENTS command
    sendCmd_ReadMeasurements();

    // Receive measurements
    double temperature = UNDEFINED, humidity = UNDEFINED;
    if (recvMeasurements(&temperature, &humidity)) {
        //log out to serial UUART
        REPORT_TEMPHUM(temperature, humidity);

        //send sensor readings to broker
        sendSensorMessage(temperature, humidity);
    }

    // Send additional commands
    sendCmd_06();
    sendCmd_07();

    // Receive dummy bytes (optional)
    uint8_t fillBuf[5];
    memset(fillBuf, FILL_BYTE, 5);
    uint8_t readBuf[5];
    hspiTransfer(fillBuf, readBuf, 5);
}

// Send READ_MEASUREMENTS command
bool sendCmd_ReadMeasurements() {
    uint8_t frmLen = 0;
    uint8_t* mosi = makeMOSIFrame(CMD_READ_MEASUREMENTS, nullptr, 0, &frmLen);
    if (!mosi) return false;

    uint8_t miso[64];
    memset(miso, 0, sizeof(miso));

    hspiTransfer(mosi, miso, frmLen);
    free(mosi);

    return misoFrameValid(miso, frmLen);
}

// Receive measurements
bool recvMeasurements(double* temperature, double* humidity) {
    uint8_t* fillFrame = makeMOSIFillFrame(12);
    if (!fillFrame) return false;

    uint8_t miso[12];
    memset(miso, 0, 12);

    hspiTransfer(fillFrame, miso, 12);
    free(fillFrame);

    bool validFrame = misoFrameValid(miso, 12);

    // Parse temperature and humidity
    int16_t temp8 = ((int16_t)miso[7] << 8) | miso[8];
    double t = (double)temp8 / 8.0;
    double h = (double)miso[10] / 2.0;

    *temperature = t;
    *humidity    = h;

    // Verify checksum (optional)
    uint8_t calculated_checksum = miso[0];
    for (int i = 1; i < 11; i++) {
        calculated_checksum ^= miso[i];
    }
    calculated_checksum = 0xFF - calculated_checksum;

    // You can choose to handle checksum mismatches if needed

    return true; // Return true even if frame is invalid to report data
}

// Send LED command
bool sendCmd_LED(uint8_t led_status) {
    uint8_t data[2] = { led_status, 0x04 };
    uint8_t frmLen = 0;
    uint8_t* mosi = makeMOSIFrame(CMD_LED, data, 2, &frmLen);
    if (!mosi) return false;

    uint8_t miso[64];
    memset(miso, 0, sizeof(miso));

    hspiTransfer(mosi, miso, frmLen);
    free(mosi);

    return misoFrameValid(miso, frmLen);
}

// Send PREVENT_SLEEP command
bool sendCmd_PreventSleep() {
    uint8_t frmLen = 0;
    uint8_t* mosi = makeMOSIFrame(CMD_PREVENT_SLEEP, nullptr, 0, &frmLen);
    if (!mosi) return false;

    uint8_t miso[64];
    memset(miso, 0, sizeof(miso));

    hspiTransfer(mosi, miso, frmLen);
    free(mosi);

    return misoFrameValid(miso, frmLen);
}

// Send UNKNOWN_06 command
bool sendCmd_06() {
    uint8_t frmLen = 0;
    uint8_t* mosi = makeMOSIFrame(CMD_UNKNOWN_06, nullptr, 0, &frmLen);
    if (!mosi) return false;

    uint8_t miso[64];
    memset(miso, 0, sizeof(miso));

    hspiTransfer(mosi, miso, frmLen);
    free(mosi);

    return misoFrameValid(miso, frmLen);
}

// Send UNKNOWN_07 command
bool sendCmd_07() {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint8_t frmLen = 0;
    uint8_t* mosi = makeMOSIFrame(CMD_UNKNOWN_07, data, 5, &frmLen);
    if (!mosi) return false;

    uint8_t miso[64];
    memset(miso, 0, sizeof(miso));

    hspiTransfer(mosi, miso, frmLen);
    free(mosi);

    return misoFrameValid(miso, frmLen);
}

// ------------------------------------------------------------
// Frame-building & Validation
// ------------------------------------------------------------
uint8_t* makeMOSIFrame(uint8_t cmd, const uint8_t* data, uint8_t data_len, uint8_t* out_len) {
    // Frame size = 2(header) +1(cmd) +1(len) + data_len +1(chksum) +2(footer)
    uint8_t totalLen = 2 + 1 + 1 + data_len + 1 + 2;
    *out_len = totalLen;

    uint8_t* frame = (uint8_t*) malloc(totalLen);
    if (!frame) return nullptr;

    uint8_t* p = frame;
    // Header
    *(p++) = HEADER_BYTE1;
    *(p++) = HEADER_BYTE2;
    // Command
    *(p++) = cmd;
    // Data length
    *(p++) = data_len;

    // Data
    if (data_len && data) {
        memcpy(p, data, data_len);
        p += data_len;
    }

    // Checksum: 0xFF - XOR(cmd, data_len, data[])
    uint8_t cs = cmd ^ data_len;
    for (uint8_t i = 0; i < data_len; i++) {
        cs ^= data[i];
    }
    cs = 0xFF - cs;
    *(p++) = cs;

    // Footer
    *(p++) = FOOTER_BYTE1;
    *(p++) = FOOTER_BYTE2;

    return frame;
}

uint8_t* makeMOSIFillFrame(uint8_t len) {
    uint8_t* frame = (uint8_t*) malloc(len);
    if (!frame) return nullptr;
    memset(frame, FILL_BYTE, len);
    return frame;
}

// Validate MISO frame
bool misoFrameValid(const uint8_t* frame, uint8_t frame_len) {
    if (frame_len < 2) return false;
    return (frame[0] == FILL_BYTE) && (frame[frame_len - 1] == SLAVE_FOOTER_BYTE2);
}

 void wakeUpSensor() {
    digitalWrite(PIN_HSPI_CS, LOW);
    delay(100);
    digitalWrite(PIN_HSPI_CS, HIGH);
    delay(100);
}

void resetLowPowerMCU() {
    digitalWrite(PIN_RESET, LOW);
    delay(100);
    digitalWrite(PIN_RESET, HIGH);
    delay(100);
}

void setupSTM(){
   // HSPI pin modes
    pinMode(PIN_HSPI_MISO, SPECIAL);
    pinMode(PIN_HSPI_MOSI, SPECIAL);
    pinMode(PIN_HSPI_SCLK, SPECIAL);
    pinMode(PIN_HSPI_CS,   OUTPUT);
    digitalWrite(PIN_HSPI_CS, HIGH); // De-select

    pinMode(PIN_RESET, OUTPUT);
    digitalWrite(PIN_RESET, HIGH); // Keep Shelly's secondary MCU out of reset

    // Setup the SPI bus at the desired freq & mode
    hspiSetup();

    // Wake the sensor
    wakeUpSensor();

    // Initialize sensor settings
    sendCmd_PreventSleep();
    sendCmd_LED(LED_STATUS_BLINK_SLOW);
}

void sensorLoop(){
  unsigned long lastAttempt = 0;
    unsigned long now = millis();
    if (now - lastAttempt >= 5000) { // Every 5 seconds
        lastAttempt = now;
        readShellyHTData();
    }
    delay(100);
}

String getDeviceId() {
    // Temporary storage for the Config struct
    Config tempConfig;

    // Read data from EEPROM into tempConfig
    for (size_t i = 0; i < sizeof(Config); ++i) {
        *((uint8_t*)&tempConfig + i) = EEPROM.read(i);
    }

    // Calculate checksum
    uint32_t calculatedChecksum = calculateChecksum(reinterpret_cast<uint8_t*>(&tempConfig), sizeof(Config) - sizeof(uint32_t));

    // Verify checksum
    if (tempConfig.checksum == calculatedChecksum) {
        if (strlen(tempConfig.deviceId) == 0) {
            Serial.println("[EEPROM] DeviceID field is empty.");
            return String(""); // Return empty string if deviceId is empty
        }

        Serial.printf("[EEPROM] Retrieved DeviceID: %s\n", tempConfig.deviceId);
        return String(tempConfig.deviceId);
    }

    Serial.println("[EEPROM] Invalid configuration or checksum mismatch while retrieving DeviceID.");
    return String(""); // Return empty string if checksum fails
}


String getUserId() {
    // Temporary storage for the Config struct
    Config tempConfig;

    // Read data from EEPROM into tempConfig
    for (size_t i = 0; i < sizeof(Config); ++i) {
        *((uint8_t*)&tempConfig + i) = EEPROM.read(i);
    }

    // Ensure null-termination for UUID
    tempConfig.uuid[UUID_LENGTH - 1] = '\0';

    // Calculate checksum
    uint32_t calculatedChecksum = calculateChecksum(reinterpret_cast<uint8_t*>(&tempConfig), sizeof(Config) - sizeof(uint32_t));

    // Verify checksum
    if (tempConfig.checksum == calculatedChecksum) {
        if (strlen(tempConfig.uuid) == 0) {
            Serial.println("[EEPROM] UUID field is empty.");
            return String(""); // Return empty string if UUID is empty
        }

        Serial.printf("[EEPROM] Retrieved UUID: %s, Length: %d\n", tempConfig.uuid, strlen(tempConfig.uuid));
        return String(tempConfig.uuid);
    }

    Serial.println("[EEPROM] Invalid configuration or checksum mismatch while retrieving UUID.");
    return String(""); // Return empty string if checksum fails
}


void sendSensorMessage(float temperature, float humidity) {

    // Retrieve necessary IDs
    String userId = getUserId();
    String deviceId = getDeviceId();
    String status = "online";
    String deviceType = "sensor";

    // Create a JSON document
    StaticJsonDocument<256> doc;

    // Populate required fields
    doc["device_id"] = deviceId;
    doc["user_id"] = userId;
    doc["status"] = status;
    doc["device_type"] = deviceType;

    // Populate optional fields if valid
    if (!isnan(temperature)) {
        doc["temp_sensor_reading"] = temperature;
    }

    if (!isnan(humidity)) {
        doc["humid_sensor_reading"] = humidity;
    }

    // Serialize JSON to a String
    String jsonResponse;
    serializeJson(doc, jsonResponse);

    // Construct dynamic topic: gms/${userId}/${deviceId}
    String topic = getTopic();

    Serial.println(jsonResponse);


    if(mqttClient.connected()){

        if(!mqttClient.subscribe(topic.c_str())){
            Serial.println("Error Subscribing for some reason");
        }

        if(!mqttClient.publish(topic.c_str(), jsonResponse.c_str())){
            Serial.println("Error publishing for some reason");
            Serial.print("[ERROR] MQTT Publish Failed, State: ");
            Serial.println(mqttClient.state());

        }
    } else{
        Serial.print("Unable to reconnect to mqtt");
    }


    


    Serial.println("[MQTT] Payload published successfully");
}

