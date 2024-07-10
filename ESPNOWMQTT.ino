// ESPNOW to MQTT Gateway made for WLED
// Made for the community by Subrata Dey
// Feel Free to Edit/Use/Reuse as you see fit

#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <Preferences.h>

// MQTT Credentials
int mqttPort = 1883; // Default MQTT port
char mqttServer[40];
char mqttUser[40];
char mqttPassword[40];

// MQTT Topics
const char* outgoingTopic = "espnow/outgoing";
const char* incomingTopicBase = "espnow/incoming";
const char* statusTopic = "espnow/status";

// ESP-NOW Configuration
const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
esp_now_peer_info_t peerInfo;

// WLED Message Structure
typedef struct {
  uint8_t program;
  uint8_t seq[4];
  uint8_t byte5;
  uint8_t button;
  uint8_t byte8;
  uint8_t byte9;
  uint8_t byte10;
  uint8_t byte11;
  uint8_t byte12;
  uint8_t byte13;
} remote_message_struct;

// Global Variables
uint32_t sequenceNumber = 0;
unsigned long startTime = millis();

// WiFi and MQTT Client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// MQTT Callback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.write(payload, length);
  Serial.println();

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("deserializeJson() failed with code: ");
    Serial.println(error.c_str());
    return;
  }

  String devicePlatform = doc["device_platform"];
  int wifiChannel = doc["channel"];

  if (wifiChannel != 0 && (wifiChannel < 1 || wifiChannel > 14)) {
    Serial.println("Invalid Wi-Fi channel");
    return;
  }

  if (devicePlatform == "wizremote") {
    int buttonCode = doc["button"];
    if (buttonCode == 0) {
      Serial.println("Invalid button code");
      return;
    }

    Serial.print("Button Code: ");
    Serial.println(buttonCode);

    remote_message_struct outgoingMessage;
    outgoingMessage.program = 0x81;

    switch (buttonCode) {
      case 1:
        outgoingMessage.program = 0x91;
        outgoingMessage.button = 1;
        break;
      case 2:
        outgoingMessage.button = 2;
        break;
      case 16:
        outgoingMessage.button = 16;
        break;
      case 17:
        outgoingMessage.button = 17;
        break;
      case 18:
        outgoingMessage.button = 18;
        break;
      case 19:
        outgoingMessage.button = 19;
        break;
      case 9:
        outgoingMessage.button = 9;
        break;
      case 8:
        outgoingMessage.button = 8;
        break;
      default:
        Serial.println("Unsupported button code for wizremote");
        return;
    }

    sequenceNumber++;
    outgoingMessage.seq[0] = sequenceNumber & 0xFF;
    outgoingMessage.seq[1] = (sequenceNumber >> 8) & 0xFF;
    outgoingMessage.seq[2] = (sequenceNumber >> 16) & 0xFF;
    outgoingMessage.seq[3] = (sequenceNumber >> 24) & 0xFF;

    outgoingMessage.byte5 = 0x20;
    outgoingMessage.byte8 = 0x01;
    outgoingMessage.byte9 = 0x64;

    if (wifiChannel != 0) {
      WiFi.setChannel(wifiChannel);
      delay(100);

      Serial.print("ESP-NOW Message Data: ");
      for (int i = 0; i < sizeof(outgoingMessage); i++) {
        Serial.print(((uint8_t*)&outgoingMessage)[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
      if (result == ESP_OK) {
        Serial.println("Sent ESP-NOW message on specific channel");
      } else {
        Serial.println("Error sending ESP-NOW message on specific channel");
      }
    } else {
      for (int i = 1; i <= 14; ++i) {
        WiFi.setChannel(i);
        delay(10);
        esp_now_send(broadcastAddress, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
      }
      Serial.println("Broadcast ESP-NOW message on all channels");
      Serial.print("Sending ESP-NOW message with data: ");
      for (int i = 0; i < sizeof(outgoingMessage); i++) {
        Serial.print(((uint8_t*)&outgoingMessage)[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  } else {
    Serial.println("Unsupported device platform:");
    Serial.println(devicePlatform);
  }
}

// ESP-NOW Data Receive Callback
void onDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", esp_now_info->src_addr[0], esp_now_info->src_addr[1], esp_now_info->src_addr[2], esp_now_info->src_addr[3], esp_now_info->src_addr[4], esp_now_info->src_addr[5]);

  String topic = String(incomingTopicBase) + "/" + macStr;
  Serial.print("Incoming ESP-NOW message on topic: ");
  Serial.println(topic);
  mqttClient.publish(topic.c_str(), (const char*)data, data_len);
  Serial.println("Sent message to MQTT");
}

// Send Status Update to MQTT
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

// Setup Function
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