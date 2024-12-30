// utils.cpp
#include "utils.h"
#include "secrets.h" // Include secrets.h for MQTT credentials
#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <HTS221Sensor.h>

// Global Variables
Config storedConfig;
HTS221Sensor sensor(&Wire);


// Initialize the sensor
void initializeSensor() {
    Serial.println("[DEBUG] Initializing HTS221 sensor...");

    // Initialize I2C with specified SDA and SCL pins
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("[DEBUG] I2C initialized successfully!");

    // Initialize the HTS221 sensor
    if (sensor.begin() != HTS221_STATUS_OK) {
        Serial.println("[ERROR] Failed to initialize HTS221 sensor!");
        while (1) delay(1000); // Halt if initialization fails
    }
    Serial.println("[DEBUG] HTS221 sensor initialized successfully!");

    // Enable the HTS221 sensor
    if (sensor.Enable() != HTS221_STATUS_OK) {
        Serial.println("[ERROR] Failed to enable HTS221 sensor!");
        while (1) delay(1000); // Halt if enabling fails
    }
    Serial.println("[DEBUG] HTS221 sensor enabled successfully!");
}


// For testing with a public broker that doesn't require TLS:
// Example: HiveMQ public broker
// MQTT_SERVER = "broker.hivemq.com"
// MQTT_PORT = 1883
const char* mqtt_publish_topic    = "/gms/user/5d62be50-ec8f-48f4-b576-a240e42db066";
const char* mqtt_subscribe_topic  = "/gms/user/5d62be50-ec8f-48f4-b576-a240e42db066";

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

    Serial.print("Temperature: ");
    Serial.print(data.temperature);
    Serial.println(" °C");

    Serial.print("Humidity: ");
    Serial.print(data.humidity);
    Serial.println(" %");

    return data;
}

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

void wakeUpSTM32() {
    Serial.println("[DEBUG] Triggering wake-up pin...");

    pinMode(16, OUTPUT); // Example GPIO pin, adjust if needed
    digitalWrite(16, HIGH);
    delay(100);  // Hold high for 100ms
    digitalWrite(16, LOW);

    Serial.println("[INFO] Wake-up signal sent via GPIO!");
}

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

void scanI2CDevices() {
  Serial.println("[DEBUG] Scanning I²C bus for devices...");

  int nDevices = 0;

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("[INFO] Found I²C device at address 0x");
      Serial.println(address, HEX);

      dumpI2CRegisters(address); // Dump registers for this device
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

void runFullDiagnostics() {
  Serial.println("\n🔍 [DEBUG] Starting Full System Diagnostics...\n");
  
  // ---------- GPIO Pin Probing ----------
  Serial.println("🟢 [GPIO] Testing All GPIO Pins...");
  for (int pin = 0; pin <= 16; pin++) {
      if (pin == 1 || pin == 3) continue; // Skip TX/RX
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
      Serial.printf("[GPIO] GPIO%d set to HIGH\n", pin);
      delay(200);
      digitalWrite(pin, LOW);
      Serial.printf("[GPIO] GPIO%d set to LOW\n", pin);
      delay(200);
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
    for (int i = 0; i < gpioPins[i]; i++) {
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
    for (int i = 0; i < gpioPins[i]; i++) {
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

void simulateButtonPress() {
    Serial.println("[TEST] Simulating Button Press on GPIO5...");
    pinMode(5, OUTPUT);
    digitalWrite(5, LOW); delay(100);
    digitalWrite(5, HIGH); delay(500);
    digitalWrite(5, LOW); delay(100);
    Serial.println("[TEST] Button Press Simulation Complete");
}

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
void scanI2CBus() {
    Serial.println("🔍 [I²C SCAN] Scanning for devices...");

    for (uint8_t address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("✅ Device found at address: 0x%02X\n", address);
        }
    }

    Serial.println("✅ [I²C SCAN COMPLETE]");
}