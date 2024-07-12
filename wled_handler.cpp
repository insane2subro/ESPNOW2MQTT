#include "wled_handler.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <esp_now.h>


extern uint32_t sequenceNumber;
extern const uint8_t broadcastAddress[]; 

// Get platform identifier from MQTT topic (No change)
String getDevicePlatformFromTopic(char* topic) {
   return "wled"; // Always return "wled" as the platform
}
// Handle platform-specific messages (WLED implementation)
void handlePlatformMessage(const String& platform, byte* payload, unsigned int length) {
    if (platform != "wled") {
        Serial.println("Unsupported platform: " + platform);
        return;
    }

    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
        Serial.print("deserializeJson() failed with code: ");
        Serial.println(error.c_str());
        return;
    }
   
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

    // Broadcast on all channels
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
