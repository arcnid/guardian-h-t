#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <time.h>

// Include your custom headers
#include "utils/utils.h"
#include "api/api.h" // Ensure this path is correct based on your project structure

// Web server on port 80
ESP8266WebServer server(80);

// Timer variables
unsigned long lastPublishTime = 0;            // Last time a message was published
const unsigned long publishInterval = 30000;  // Interval to publish messages (in ms)

// MQTT reconnection variables
unsigned long lastReconnectAttempt = 5000; // 5 seconds

const unsigned long reconnectInterval = 10000;
// WiFi reconnection variables
unsigned long lastWifiRetryAttempt = 0;
const unsigned long wifiRetryInterval = 30000; // 30 seconds


unsigned long lastHeartbeatTime = 0;
const unsigned long heartbeatInterval = 60000; // 1 minute
// Function to start Access Point
void startAccessPoint() {
  // Start the AP
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("Guardian_GMS_Sensor", "");

  // Log out the IP that was assigned to the device
  Serial.print("Access point IP: ");
  Serial.println(WiFi.softAPIP());

  // Turn LED on to indicate AP mode
  digitalWrite(SHELLY_BUILTIN_LED, LOW);
}

void synchronizeTime() {
  Serial.println("Synchronizing system time...");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov"); // UTC Timezone

  struct tm timeinfo;
  // Wait for time synchronization
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Waiting for time synchronization...");
    delay(1000);
  }

  // Buffer to hold the formatted time string
  char timeStr[64];
  // Format the time using strftime
  strftime(timeStr, sizeof(timeStr), "Current time: %A, %B %d %Y %H:%M:%S", &timeinfo);
  // Print the formatted time
  Serial.println(timeStr);
}

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);
  Serial.println("\nFirmware Started");

  // Initialize EEPROM
  EEPROM.begin(512); // Adjust size as needed

  // Optional: Clear EEPROM (Uncomment if needed)
  // clearEEPROM();

  // Set the LED pin to output mode and turn it off initially
  pinMode(SHELLY_BUILTIN_LED, OUTPUT);
  digitalWrite(SHELLY_BUILTIN_LED, HIGH); // Ensure LED is off initially

  // Check for stored WiFi credentials
  if (checkForWifiAndUser()) {
    // Attempt to connect using EEPROM credentials
    if (connect(String(storedConfig.ssid), String(storedConfig.password))) {
      Serial.println("Connected to WiFi successfully.");
      digitalWrite(SHELLY_BUILTIN_LED, HIGH); // Turn LED off after successful connection
    } else {
      Serial.println("WiFi connection failed, starting Access Point...");
      startAccessPoint();
    }
  } else {
    Serial.println("No Credentials Found, Starting Access Point...");
    startAccessPoint();
  }

  // Synchronize system time if connected
  if (WiFi.status() == WL_CONNECTED) {
    synchronizeTime();
  } else {
    Serial.println("WiFi not connected. Unable to synchronize time.");
  }

  // Initialize API routes
  setupApiRoutes(server);

  // Start the Web Server
  server.begin();
  Serial.println("Web Server Started");

  // Initialize MQTT only if user exists and WiFi is connected
  if (doesUserExist && WiFi.status() == WL_CONNECTED) {
    if (connectToMQTT()) {
      Serial.println("MQTT Connected Successfully.");
    } else {
      Serial.println("Failed to Connect to MQTT Broker.");
    }
  }

  initializeSensor();

  

}

void loop() {
  // Handle incoming client requests
  server.handleClient();




  // If we have stored credentials (user exists) but WiFi is not connected, periodically attempt to reconnect
  if (doesUserExist && (WiFi.status() != WL_CONNECTED)) {
    unsigned long now = millis();
    if (now - lastWifiRetryAttempt > wifiRetryInterval) {
      lastWifiRetryAttempt = now;
      Serial.println("Attempting to reconnect to WiFi...");
      if (connect(String(storedConfig.ssid), String(storedConfig.password))) {
        Serial.println("Reconnected to WiFi successfully.");
        digitalWrite(SHELLY_BUILTIN_LED, HIGH); // Turn LED off

        // Synchronize system time again after reconnecting
        synchronizeTime();

        // If MQTT is not connected, try connecting again
        if (doesUserExist && !mqttClient.connected()) {
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

  // If MQTT should be maintained and WiFi is connected
  if (doesUserExist && (WiFi.status() == WL_CONNECTED)) {
    // Ensure MQTT connection is maintained
    if (!mqttClient.connected()) {
      unsigned long now = millis();
      if (now - lastReconnectAttempt > reconnectInterval) {
        lastReconnectAttempt = now;
        Serial.println("Reconnecting to MQTT Broker...");
        if (connectToMQTT()) {
          Serial.println("Reconnected to MQTT Broker.");
        } else{
          Serial.println("Failed to connect to broker");
          delay(2000);
        }
      }
    }
    mqttClient.loop();

    // Publish status message at defined intervals
    unsigned long currentMillis = millis();
    if (currentMillis - lastPublishTime >= publishInterval) {
      lastPublishTime = currentMillis;
      String status = "Device is online. IP: " + WiFi.localIP().toString();
      publishMessage(mqtt_publish_topic, status.c_str());
    }
  }

  unsigned long currentMillis = millis();
    if (currentMillis - lastHeartbeatTime >= heartbeatInterval) {
        lastHeartbeatTime = currentMillis;
        sendHeartbeat();
    }



  // Optionally, log free heap memory
  // Serial.printf("Free Heap: %u bytes\n", ESP.getFreeHeap());
}
