#ifndef WLED_HANDLER_H
#define WLED_HANDLER_H

#include <Arduino.h>

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

extern const uint8_t broadcastAddress[]; 

// Function prototypes (declarations)
String getDevicePlatformFromTopic(char* topic);
void handlePlatformMessage(const String& platform, byte* payload, unsigned int length);

#endif
