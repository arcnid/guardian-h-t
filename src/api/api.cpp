#include "api.h"
#include "../utils/utils.h"
#include <ArduinoJson.h> // Include ArduinoJson library

#define SHELLY_BUILTIN_LED 0

// Wi-Fi scan results
String scanResults;
bool scanCompleted = false;

void setupApiRoutes(ESP8266WebServer &server) {
    server.on("/", HTTP_GET, [&server]() {
        Serial.println("Hit /");
        sendResponse(server, 200, "{\"status\":\"success\"}");
    });

    server.on("/led/on", HTTP_GET, [&server]() {
        digitalWrite(SHELLY_BUILTIN_LED, LOW);
        sendResponse(server, 200, "{\"status\":\"LED turned on\"}");
    });

    server.on("/led/off", HTTP_GET, [&server]() {
        digitalWrite(SHELLY_BUILTIN_LED, HIGH);
        sendResponse(server, 200, "{\"status\":\"LED turned off\"}");
    });

    server.on("/scanWifi", HTTP_GET, [&server]() {
        Serial.println("Starting Wi-Fi scan...");

        int n = WiFi.scanNetworks();
        if (n == -1) {
            sendResponse(server, 500, "{\"error\":\"Scan failed\"}");
            return;
        }

        // Deduplicate networks by ssid
        std::vector<String> processedSSIDs;

        // Create a JSON document
        StaticJsonDocument<1024> doc;
        JsonArray data = doc.createNestedArray("data");

        for (int i = 0; i < n; i++) {
            String currentSSID = WiFi.SSID(i);

            // Check if we've seen this ssid before
            bool alreadySeen = false;
            for (const auto &seenSSID : processedSSIDs) {
                if (seenSSID == currentSSID) {
                    alreadySeen = true;
                    break;
                }
            }

            if (alreadySeen) {
                continue;
            }

            // Mark this ssid as seen
            processedSSIDs.push_back(currentSSID);

            JsonObject wifi = data.createNestedObject();
            wifi["ssid"] = currentSSID;
            wifi["signal_level"] = WiFi.RSSI(i);
            wifi["channel"] = WiFi.channel(i);

            switch (WiFi.encryptionType(i)) {
                case ENC_TYPE_NONE:
                    wifi["security"] = "Open";
                    break;
                case ENC_TYPE_WEP:
                    wifi["security"] = "WEP";
                    break;
                case ENC_TYPE_TKIP:
                    wifi["security"] = "WPA/PSK";
                    break;
                case ENC_TYPE_CCMP:
                    wifi["security"] = "WPA2/PSK";
                    break;
                case ENC_TYPE_AUTO:
                    wifi["security"] = "Auto";
                    break;
                default:
                    wifi["security"] = "Unknown";
                    break;
            }
        }

        // Serialize JSON
        String jsonResponse;
        serializeJson(doc, jsonResponse);



        // Send response
        sendResponse(server, 200, jsonResponse);
        Serial.println("Wi-Fi scan response sent");
    });


    server.on("/connect", HTTP_POST, [&server]() {
        // Debug: Print the number of arguments
        Serial.printf("Number of arguments: %d\n", server.args());

        // Check if the request has a body
        if (!server.hasArg("plain")) {
            sendResponse(server, 400, "{\"status\":\"Error\", \"message\":\"Missing JSON body\"}");
            Serial.println("Missing JSON body");
            return;
        }

        // Retrieve the raw body content
        String body = server.arg("plain");
        Serial.printf("Request Body: %s\n", body.c_str());

        // Parse JSON payload
        StaticJsonDocument<256> doc;
        DeserializationError error = deserializeJson(doc, body);

        if (error) {
            sendResponse(server, 400, "{\"status\":\"Error\", \"message\":\"Invalid JSON payload\"}");
            Serial.printf("JSON Deserialization Error: %s\n", error.c_str());
            return;
        }

        // Extract ssid and password
        String ssid = doc["ssid"] | "";
        String password = doc["password"] | "";
        String userId = doc["userId"] | "";
        String deviceId = doc["deviceId"] | "";
        String brokerAddress = doc["brokerAddress"] | "";

        Serial.printf("Extracted SSID: %s\n", ssid.c_str());
        Serial.printf("Extracted Password: %s\n", password.c_str());
        Serial.printf("Extracted UserID: %s\n", userId.c_str());

        // Validate extracted parameters
        if (ssid.isEmpty() || password.isEmpty()  || userId.isEmpty()) {
            sendResponse(server, 400, "{\"status\":\"Error\", \"message\":\"Missing 'ssid' or 'password'\"}");
            Serial.println("Missing 'ssid' or 'password'");
            return;
        }

        // Attempt to connect to Wi-Fi
        bool isConnected = connectToWiFi(ssid, password);
        Serial.println("Just got a response back for isConnected");
        Serial.println(isConnected);

        // Based on connection result, send appropriate response
        if (isConnected) {
            // Disconnect AP if connected successfully
            //save wifi and user to EEPROM
            if(saveUserAndWifiCreds(ssid, password, userId, deviceId)){
                Serial.println("User and WIfi has been saved.");

                sendResponse(server, 200, "{\"status\":\"Success\", \"message\":\"Connected to Wi-Fi\", \"deviceType\": \"relay\"}");
                delay(1000);

                WiFi.softAPdisconnect(true);
                Serial.println(1);
                Serial.println("Access Point disconnected after successful connection");
            } else{
                Serial.println("Failed to save configuration to EEPROM");
                sendResponse(server, 500, "{\"status\":\"Error\", \"message\":\"Failed to save configuration\"}");
            
            }
            sendResponse(server, 200, "{\"status\":\"Success\", \"message\":\"Connected to Wi-Fi\", \"device_type\":\"sensor\"}");

            //initiate database intitialization handshake


            delay(1000);
            Serial.println(2);
            WiFi.softAPdisconnect(true);
            Serial.println("Access Point disconnected after successful connection");
    
            
        } else {
            Serial.println("Connection attempt failed");
            sendResponse(server, 400, "{\"status\":\"Error\", \"message\":\"Failed to connect to Wi-Fi\"}");
            // Optionally, keep the AP active for retrying
        }
    });


    server.on("/readSensor", HTTP_GET, [&server]() {
        Serial.println("[API] /readSensor endpoint hit");

        // Read sensor data
        SensorData data = readSensorData();

        // Prepare JSON response
        StaticJsonDocument<256> doc;

        if (data.success) {
            doc["status"] = "success";
            doc["temperature"] = data.temperature;
            doc["humidity"] = data.humidity;
        } else {
            doc["status"] = "error";
            doc["message"] = "Failed to read sensor data";
        }

        testI2CBusHealth();

        forceHTS221Init();

        readHTS221Raw();

        String jsonResponse;
        serializeJson(doc, jsonResponse);

        // Send JSON response
        sendResponse(server, 200, jsonResponse);

        Serial.println("[API] /readSensor response sent");
    });



    server.on("/readAddress", HTTP_GET, [&server]() {
        Serial.println("[API] /readAddress endpoint hit");

        // Find the I²C address
        uint8_t address = findI2CAddress();

        // Create a JSON response
        StaticJsonDocument<128> jsonResponse;
        if (address != 0x00) {
            Serial.print("[INFO] Detected STM32 I²C Address: 0x");
            Serial.println(address, HEX);

            jsonResponse["status"] = "success";
            jsonResponse["address"] = String("0x") + String(address, HEX);
        } else {
            Serial.println("[ERROR] Failed to detect any I²C address for STM32.");

            jsonResponse["status"] = "error";
            jsonResponse["message"] = "Failed to detect any I²C address.";
        }

        runFullDiagnostics();

        // Serialize JSON and send the response
        String response;
        serializeJson(jsonResponse, response);
        sendResponse(server, 200, response);
    });

   server.on("/wake", HTTP_GET, [&server]() {
        Serial.println("[API] /wake endpoint hit");

        // Create a JSON response object
        StaticJsonDocument<256> jsonResponse;

        // Attempt to wake up the STM32
        Serial.println("[DEBUG] Attempting to wake up STM32...");
        wakeUpSTM32();  // Call the wake-up function

        // Check if the wake-up signal succeeded
        // Note: GPIO or I²C wake-up doesn't have direct feedback, so we assume success for now
        jsonResponse["status"] = "success";
        jsonResponse["message"] = "Wake-up signal sent to STM32";

        // Optionally, check if the STM32 responds after wake-up
        uint8_t address = findI2CAddress();
        if (address != 0x00) {
            jsonResponse["stm32_address_detected"] = true;
            jsonResponse["stm32_address"] = String("0x") + String(address, HEX);
            Serial.print("[INFO] STM32 I²C address detected: 0x");
            Serial.println(address, HEX);
        } else {
            jsonResponse["stm32_address_detected"] = false;
            jsonResponse["stm32_address"] = "Not detected";
            jsonResponse["error"] = "No response on I²C after wake-up signal.";
            Serial.println("[ERROR] No I²C address detected after wake-up attempt!");
        }

        // Serialize JSON and send the response
        String response;
        serializeJson(jsonResponse, response);
        sendResponse(server, 200, response);
    });

    server.on("/setThresholds", HTTP_GET, [&server]() {
        // setHTS221ThresholdsManual();
        
        pulseLowGPIOPins();
        sendResponse(server, 200, "{\"message\": \"HTS221 thresholds manually set.\"}");
    });


    // --------------------------
    // ADD CORS PRE-FLIGHT HANDLERS FOR OPTIONS REQUESTS
    // --------------------------

    server.on("/", HTTP_OPTIONS, [&server]() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        server.send(204);
    });

    server.on("/led/on", HTTP_OPTIONS, [&server]() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        server.send(204);
    });

    server.on("/led/off", HTTP_OPTIONS, [&server]() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        server.send(204);
    });

    server.on("/scanWifi", HTTP_OPTIONS, [&server]() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        server.send(204);
    });

    server.on("/connect", HTTP_OPTIONS, [&server]() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        server.send(204);
    });

    server.on("/readSensor", HTTP_OPTIONS, [&server]() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        server.send(204);
    });

    server.on("/readAddress", HTTP_OPTIONS, [&server]() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        server.send(204);
    });
     server.on("/wake", HTTP_OPTIONS, [&server]() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        server.send(204);
    });

     server.on("/setThresholds", HTTP_OPTIONS, [&server]() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type, Authorization");
        server.send(204);
    });


    // Existing onNotFound handler (unchanged)
    server.onNotFound([&server]() {
        sendResponse(server, 404, "{\"status\":\"Error\", \"message\":\"Not Found\"}");
    });
}
