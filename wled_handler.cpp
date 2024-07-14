#include "wled_handler.h"
#include <WiFi.h>
#include <esp_now.h>


extern uint32_t sequenceNumber;
extern const uint8_t broadcastAddress[]; 
extern void logEspNowResult(esp_err_t result, const String& command, int channel); // Declare the function

// WLED-specific message handler
void handleWledMessage(DynamicJsonDocument& doc) {
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
            outgoingMessage.program = 0x91; // Toggle on/off (including nightlight mode)
            outgoingMessage.button = 1;
            break;
        case 2:
            outgoingMessage.button = 2; // Brightness up
            break;
        case 16:
            outgoingMessage.button = 16; // Mode up
            break;
        case 17:
            outgoingMessage.button = 17; // Mode down
            break;
        case 18:
            outgoingMessage.button = 18; // Speed up
            break;
        case 19:
            outgoingMessage.button = 19; // Speed down
            break;
        case 9:
            outgoingMessage.button = 9;  // Color up
            break;
        case 8:
            outgoingMessage.button = 8;  // Color down
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

    int channel = doc["channel"]; // Get the channel from the JSON payload

    // Check if the channel is valid
    if (channel >= 1 && channel <= 14) {
        // Send on the specified channel
        WiFi.setChannel(channel);
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
        logEspNowResult(result, String(buttonCode), channel);
    } else {
        // Broadcast on all channels if the channel is invalid (or 0)
        esp_err_t result = ESP_OK; // Assume success initially
        for (int i = 1; i <= 14; ++i) {
            WiFi.setChannel(i);
            delay(10);
            result = esp_now_send(broadcastAddress, (uint8_t*)&outgoingMessage, sizeof(outgoingMessage));
            if (result != ESP_OK) {
                break; // Stop if an error occurs on any channel
            }
        }
        // Log the result only once after the loop
        logEspNowResult(result, String(buttonCode), 0); // 0 indicates broadcast
    }
}


