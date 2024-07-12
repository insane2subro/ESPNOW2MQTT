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

// MQTT Credentials
char mqttServer[40];
int mqttPort = 1883; 
char mqttUser[40];
char mqttPassword[40];

// MQTT Topics
const char* outgoingTopic = "espnow/outgoing";
const char* incomingTopicBase = "espnow/incoming";
const char* statusTopic = "espnow/status";

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
        // ... (WLED-specific message handling - from original code)
        int buttonCode = doc["button"];
        // ... (rest of WLED code) 
        
    } else if (platform == "other") {
        // Handle arbitrary ESP-NOW messages
        if (!doc.containsKey("command")) {
            Serial.println("Missing 'command' field in JSON payload.");
            return;
        }

        String command = doc["command"].as<String>();
        int channel = doc["channel"];

        if (channel < 1 || channel > 14) { 
            Serial.println("Invalid channel. Broadcasting on all channels.");
            for (int i = 1; i <= 14; ++i) {
                WiFi.setChannel(i);
                delay(10);
                const uint8_t* commandBytes = reinterpret_cast<const uint8_t*>(command.c_str());
                esp_now_send(broadcastAddress, commandBytes, command.length());
            }
            Serial.println("Broadcast ESP-NOW message on all channels");
        } else {
            WiFi.setChannel(channel);
            const uint8_t* commandBytes = reinterpret_cast<const uint8_t*>(command.c_str());
            esp_now_send(broadcastAddress, commandBytes, command.length());
            Serial.printf("Sent ESP-NOW command: %s on channel %d\n", command.c_str(), channel);
        }

    } else {
        Serial.println("Unsupported platform: " + platform);
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

        if (mqttClient.connect("ESP32Client", custom_mqtt_user.getValue(), custom_mqtt_password.getValue())) {
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

    // Send Initial Status Update
    sendStatusUpdate();
}

// Loop Function
void loop() {
    mqttClient.loop();

    // Send Status Updates Periodically
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 300000) {
        sendStatusUpdate();
        lastStatusTime = millis();
    }
}
