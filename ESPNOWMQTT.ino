// ESPNOW to MQTT Gateway made for WLED
// Made for the community by Subrata Dey
// Feel Free to Edit/Use/Reuse as you see fit

#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include "wled_handler.h" 
#include "esp_task_wdt.h" // To reset the ESP incase it stops responding

// MQTT Credentials
char mqttServer[40];
int mqttPort = 1883; 
char mqttUser[40];
char mqttPassword[40];

// MQTT Topics
const char* outgoingTopic = "espnow/outgoing";
const char* incomingTopicBase = "espnow/incoming";
const char* statusTopic = "espnow/status";
// LWT Message
const char* lwtTopic = "espnow/status"; // Same topic as regular status updates
const char* lwtMessage = "offline";

// ESP-NOW Configuration
const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

// Global Variables 
unsigned long startTime = millis();
uint32_t sequenceNumber = 0; // For WLED message sequencing

// WiFi and MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// MQTT Callback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Message arrived on topic: %s\n", topic);
  Serial.printf("Message length: %u bytes\n", length);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.printf("JSON deserialization failed: %s\n", error.c_str());
    return;
  }

  // Extract Device Platform (or Determine Default)
  String devicePlatform = determineDevicePlatform(doc);

  if (devicePlatform == "unknown") {
    Serial.println("Device platform not specified or unknown.");
    return; 
  }

  // Handle Platform-Specific Messages
  handlePlatformMessage(devicePlatform, doc); // Pass the parsed JSON object directly
}

// Helper function to determine device platform
String determineDevicePlatform(DynamicJsonDocument& doc) {
    if (doc.containsKey("device_platform")) {
        return doc["device_platform"].as<String>();
    } else {
        return "other"; // Default platform is now "other"
    }
}

// Handle platform-specific messages (WLED and "other" implementation)
void handlePlatformMessage(const String& platform, DynamicJsonDocument& doc) {
    if (platform == "wled") {
        handleWledMessage(doc); // Call the WLED-specific handler
    } else if (platform == "other") {
        // Handle arbitrary ESP-NOW messages
        if (!doc.containsKey("command")) {
            Serial.println("Missing 'command' field in JSON payload.");
            return;
        }

        String command = doc["command"].as<String>();
        int channel = doc["channel"].as<int>();

        // Validate channel (0 means broadcast on all channels)
        bool broadcast = (channel < 1 || channel > 14);
        if (broadcast) {
            Serial.println("Invalid channel. Broadcasting on all channels.");
            channel = 0; // Set to 0 for status update
        }

        const uint8_t* commandBytes = reinterpret_cast<const uint8_t*>(command.c_str());
        esp_err_t result = ESP_OK; // Assume success initially

        if (broadcast) {
            // Broadcast on all channels
            for (int i = 1; i <= 14; ++i) {
                WiFi.setChannel(i);
                delay(10);

                // Check result of each send operation
                result = esp_now_send(broadcastAddress, commandBytes, command.length());
                if (result != ESP_OK) {
                    break; // Stop if an error occurs on any channel
                }
            }
            Serial.println("Broadcast ESP-NOW message on all channels");
        } else {
            // Send on specific channel
            WiFi.setChannel(channel);
            result = esp_now_send(broadcastAddress, commandBytes, command.length());
        }

        // Log the result only once after the loop
        logEspNowResult(result, command, channel); 
    } else {
        Serial.println("Unsupported platform: " + platform);
    }
}

// Helper function to log ESP-NOW send results
void logEspNowResult(esp_err_t result, const String& command, int channel) {
    if (result == ESP_OK) {
        Serial.printf("Sent ESP-NOW command: %s on channel %d\n", command.c_str(), channel);

        // Create status message
        DynamicJsonDocument statusDoc(256);
        statusDoc["platform"] = "other";
        statusDoc["command"] = command;
        statusDoc["channel"] = channel;
        statusDoc["status"] = "success";

        // Serialize and publish status message
        String statusMessage;
        serializeJson(statusDoc, statusMessage);
        mqttClient.publish(statusTopic, statusMessage.c_str());
    } else {
        Serial.printf("Error sending ESP-NOW command: %s on channel %d (Error code: %d)\n", command.c_str(), channel, result);

        // Create status message
        DynamicJsonDocument statusDoc(256);
        statusDoc["platform"] = "other";
        statusDoc["command"] = command;
        statusDoc["channel"] = channel;
        statusDoc["status"] = "failure";
        statusDoc["error_code"] = result;

        // Serialize and publish status message
        String statusMessage;
        serializeJson(statusDoc, statusMessage);
        mqttClient.publish(statusTopic, statusMessage.c_str());
    }
}

// ESP-NOW Data Receive Callback
void onDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    char macStr[18]; // Mac Address of the ESP which sent the message
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", esp_now_info->src_addr[0], esp_now_info->src_addr[1], esp_now_info->src_addr[2], esp_now_info->src_addr[3], esp_now_info->src_addr[4], esp_now_info->src_addr[5]);

    String topic = String(incomingTopicBase) + "/" + macStr;
    Serial.print("Incoming ESP-NOW message on topic: ");
    Serial.println(topic);
    
    // Attempt to parse JSON
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, (const char*)data, data_len);

    if (!error) {
        // If it's valid JSON, publish as a string
        String jsonString;
        serializeJson(doc, jsonString);
        mqttClient.publish(topic.c_str(), jsonString.c_str(), false);
        Serial.println("Published JSON to MQTT: " + jsonString);
    } else {
        // If not valid JSON, publish as raw data
        mqttClient.publish(topic.c_str(), data, data_len, false);
        Serial.println("Published raw data to MQTT");
    }
}

void sendStatusUpdate() {
    DynamicJsonDocument doc(256);

    unsigned long uptimeSeconds = (millis() - startTime) / 1000;
    doc["uptime"] = uptimeSeconds;

    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[13];
    snprintf(macStr, sizeof(macStr), "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    doc["mac"] = macStr;

    long rssi = WiFi.RSSI();
    doc["rssi"] = rssi;

    String output;
    serializeJson(doc, output);
    mqttClient.publish(statusTopic, output.c_str());

    Serial.print("Sent status update: ");
    Serial.println(output);
}


void setup() {
    Serial.begin(115200);

    // Initialize Preferences
    Preferences preferences;
    if (!preferences.begin("mqtt_settings", false)) {
        Serial.println("Failed to initialize preferences");
        return;
    }

    // Load MQTT Credentials from Preferences
    mqttPort = preferences.getInt("mqttPort", mqttPort);
    preferences.getString("mqttServer", mqttServer, sizeof(mqttServer));
    preferences.getString("mqttUser", mqttUser, sizeof(mqttUser));
    preferences.getString("mqttPassword", mqttPassword, sizeof(mqttPassword));
    
    // WiFiManager Setup
    WiFiManager wm;

    // Add Custom Parameters for MQTT
    WiFiManagerParameter custom_mqtt_server("server", "MQTT Server", mqttServer, 40);
    char mqttPortStr[6];
    snprintf(mqttPortStr, sizeof(mqttPortStr), "%d", mqttPort);
    WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqttPortStr, 6);
    WiFiManagerParameter custom_mqtt_user("user", "MQTT User", mqttUser, 40);
    WiFiManagerParameter custom_mqtt_password("password", "MQTT Password", mqttPassword, 40);

    wm.addParameter(&custom_mqtt_server);
    wm.addParameter(&custom_mqtt_port);
    wm.addParameter(&custom_mqtt_user);
    wm.addParameter(&custom_mqtt_password);

    // Save Config Callback
    wm.setSaveConfigCallback([&custom_mqtt_server, &custom_mqtt_port, &custom_mqtt_user, &custom_mqtt_password]() {
        Preferences preferences;
        if (preferences.begin("mqtt_settings", false)) {
            // Save MQTT Credentials to Preferences
            preferences.putInt("mqttPort", atoi(custom_mqtt_port.getValue()));
            preferences.putString("mqttServer", custom_mqtt_server.getValue());
            preferences.putString("mqttUser", custom_mqtt_user.getValue());
            preferences.putString("mqttPassword", custom_mqtt_password.getValue());

            preferences.end();
        } else {
            Serial.println("Failed to save preferences");
        }
    });

    WiFi.setSleep(false);

    // Print Initial WiFiManager Settings
    Serial.println("\nInitial WiFiManager Settings:");
    Serial.print("Server: "); Serial.println(mqttServer);
    Serial.print("Port: "); Serial.println(mqttPort);
    Serial.print("User: "); Serial.println(mqttUser);
    Serial.println("Password: <hidden>");

    if (!wm.autoConnect("ESPNowGateway")) {
        Serial.println("Failed to connect and hit timeout");
        ESP.restart();
        delay(1000);
    }
    preferences.end();

    Serial.println("Connected to WiFi");

    // Print Updated WiFiManager Settings
    Serial.println("\nUpdated WiFiManager Settings:");
    Serial.print("Server: "); Serial.println(custom_mqtt_server.getValue());
    Serial.print("Port: "); Serial.println(custom_mqtt_port.getValue());
    Serial.print("User: "); Serial.println(custom_mqtt_user.getValue());
    Serial.println("Password: <hidden>");

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(onDataRecv);

    // Register Broadcast Peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    // Initialize MQTT Client
    mqttClient.setServer(custom_mqtt_server.getValue(), atoi(custom_mqtt_port.getValue()));
    mqttClient.setCallback(callback);

    // Connect to MQTT Broker
    while (!mqttClient.connected()) {
        Serial.println("Connecting to MQTT...");
        Serial.print("MQTT Server: "); Serial.println(custom_mqtt_server.getValue());
        Serial.print("MQTT Port: "); Serial.println(custom_mqtt_port.getValue());
        Serial.print("MQTT User: "); Serial.println(custom_mqtt_user.getValue());

        if (mqttClient.connect("ESP32Client", custom_mqtt_user.getValue(), custom_mqtt_password.getValue(), lwtTopic, 0, true, lwtMessage)) {
            Serial.println("Connected to MQTT");
            Serial.print("Subscribed to MQTT Topic: ");
            Serial.println(outgoingTopic);
            mqttClient.subscribe(outgoingTopic);
        } else {
            Serial.print("Failed to connect to MQTT, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
    // Watchdog Timer Configuration
    const int WDT_TIMEOUT = 30000; // Timeout in milliseconds (30 seconds)
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT,
        .idle_core_mask = 0,
        .trigger_panic = true,  
    };

    // Deinitialize if the watchdog timer is already running
    esp_task_wdt_deinit(); 

    // Enable and configure the watchdog timer
    esp_err_t err = esp_task_wdt_init(&wdt_config);

    // Add the current task (loop) to be monitored
    if (err == ESP_OK) {
        esp_task_wdt_add(NULL); 
    }
    // Send Initial Status Update
    sendStatusUpdate();
}

// Function to reconnect to MQTT broker
void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    // Load MQTT Credentials from Preferences (inside the loop)
    Preferences preferences;
    preferences.begin("mqtt_settings", false);
    int mqttPort = preferences.getInt("mqttPort", 1883);
    char mqttServer[40];
    char mqttUser[40];
    char mqttPassword[40];
    preferences.getString("mqttServer", mqttServer, sizeof(mqttServer));
    preferences.getString("mqttUser", mqttUser, sizeof(mqttUser));
    preferences.getString("mqttPassword", mqttPassword, sizeof(mqttPassword));
    preferences.end();

    // Attempt to connect with the loaded credentials and unique client ID
    if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword, lwtTopic, 0, true, lwtMessage)) {
      Serial.println("connected");
      mqttClient.subscribe(outgoingTopic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// Loop Function
void loop() {
    if (!mqttClient.connected()) {
        reconnect(); // Attempt to reconnect
    }
    mqttClient.loop();

    // Send Status Updates Periodically
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 300000) {
        sendStatusUpdate();
        lastStatusTime = millis();
    }

    if (ESP.getFreeHeap() < 10000) { // Example threshold: 10 KB
        Serial.println("WARNING: Low memory!");
        mqttClient.publish("espnow/status", "memory low");
    }
    // Feed the Watchdog Timer
    esp_task_wdt_reset(); 
}
