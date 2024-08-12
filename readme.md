# ESP32 MQTT to ESP-NOW Gateway

I intended to make it work with my WLED but it's open enough for you to pass ESPNOW messages to MQTT and MQTT to ESPNOW hence acting as a gateway. 

## Features

*   **MQTT Control:** Send commands to any device using simple JSON payloads over MQTT.
*   **Wi-Fi Channel Selection:** Optionally target specific Wi-Fi channels or broadcast commands to all devices in range.
*   **ESP-NOW Communication:**
    *   Efficiently transmit control commands using ESP-NOW.
    *   Receive status updates (e.g., online/offline) from devices.
*   **MQTT Status Updates:** Receive periodic updates on the gateway's uptime, MAC address, and Wi-Fi signal strength.
*   **WiFiManager Configuration:** Easily set up Wi-Fi and MQTT credentials using a captive portal

## Hardware Requirements

-   ESP32 development board (e.g., ESP32-DevKitC, NodeMCU-32S)
-   WLED devices (ESP8266 or ESP32 based) (if you need to control WLED)

## Software Requirements

-   Arduino IDE (or PlatformIO)
-   Libraries:
    *   PubSubClient
    *   ArduinoJson
    *   WiFiManager

## Installation

1.  **Clone/Download:** Get the code from this repository.
2.  **Install Libraries:** Use the Arduino Library Manager to install the required libraries.
3.  **Upload:** Compile and upload the `ESPNOWMQTT.ino` sketch to your ESP32.
4.  **Configure:**
    *   The first time you run it, the ESP32 will create a Wi-Fi network named "ESPNowGateway."
    *   Connect to this network and open a web browser to access the configuration portal.
    *   Enter your Wi-Fi credentials and MQTT broker details (address, port, username, password).
    *   Save the configuration.

## Usage

## MQTT Commands
### WLED via WiZ Remote Protocol
Publish JSON messages to the topic `**espnow/outgoing**`:
```json
{"device_platform": "wiz_remote", "button": "button_code", "channel": "wifi_channel"}
```
For WLED specifically using WizRemote protocol, where the 'button' is the button code, the 'channel' is the wifi channel (channel 0 for all wifi channels).
Example
```json
{"button": 2, "channel": 0, "device_platform": "wiz_remote"}
```
-   **`device_platform`:** Always "wiz_remote" for WLED control via Wiz Remote Commands.
-   **`button`:**  WLED button code (see WLED docs):
    -   1: ON
    -   2: OFF
    -   3: Nightlight
    -   8: Brightness Down
    -   9: Brightness Up
    -   16: Button 1
    -   17: Button 2
    -   18: Button 3
    -   19: Button 4
-   **`channel`:** (Optional) Wi-Fi channel (1-14) or 0 for broadcast.

### WLED via JSON Remote
In the latest 0.15 version of WLED the JSON Remote was introduced. All you need to do is edit one of the ir.json files from WLED (or create your own). Add a corresponding button number to the function you want. 
For example:
```json
"2": {
    "label": "On",
    "pos": "1x4",
    "cmd": "T=1"
}
```
Rename the file 'remote.json' and upload it to your WLED instance 'ipaddresss/edit'. Reboot your WLED and you are good to go.

Publish JSON messages to the topic `**espnow/outgoing**`:
```json
{"device_platform": "json_remote", "button": "button_code", "channel": "wifi_channel"}
```
WLED will match the button number with the remote.json file and do the corresponding action.

### OTHERS
Publish JSON messages to the topic `**espnow/outgoing**`:
Send a MQTT message and include whatever you want to broadcast in the "command" key. Set Wifi channel on the channel key or keep it 0 for all channels. device_pltform has to be others for now as I work to integrate other devices natively.
```json
{"command": 2, "channel": 0, "device_platform": "other"}
```
or send a JSON like:
```json
{
  "command": {"action": "toggle", "device": "light1"},
  "channel": 6,
  "device_platform": "other"
}
```
Here the whole JSON in the command key is sent as is.

### MQTT Status Updates

Subscribe to `**espnow/status**` to receive the status of the Device, Last Messages Sent(Success/Failure) and LWT:
Status:
```json
{
"uptime": "seconds",
"mac": "gateway_mac_address",
"rssi": "wifi_signal_strength"
}
```
Confirmations:
```json
{
"platform":"other",
"command":"2",
"channel":0,
"status":"success"
}
```

## MQTT Incoming Messages (from ESPNow)

Subscribe to `**espnow/incoming/**` to receive messages from any ESPNow devices broadcasting messages.
The ESPNOW messages are formatted in JSON:
```json
{"encoding":<encoding>, "data":<MESSAGE>, "device_mac":<mac of the broadcasting device>, protocol":"ESPNOW"}
```
**Example:**
```json
{"encoding":"binary","data":"gREAAAAgAgFkIPs/XA==","device_mac":"b4:e6:2d:97:f6:51","protocol":"ESPNOW"}
```
 

## Home Assistant Integration (Optional)
**For WLED**

You can integrate this gateway with Home Assistant by creating MQTT buttons/switches or lights that send the appropriate commands. 
Configuration.yaml example for Wiz Remote:
```yaml
  button:
    # WLED Buttons
    - name: "WLED ON"
      unique_id: "wled_on_button"
      command_topic: "espnow/outgoing"
      payload_press: '{"device_platform": "wiz_remote", "button": 1, "channel": 0}' # Broadcast ON

    - name: "WLED OFF"
      unique_id: "wled_off_button"
      command_topic: "espnow/outgoing"
      payload_press: '{"device_platform": "wiz_remote", "button": 2, "channel": 0}' # Broadcast OFF
   
    - name: "WLED 1"
      unique_id: "wled_1_button"
      command_topic: "espnow/outgoing"
      payload_press: '{"device_platform": "wiz_remote", "button": 16, "channel": 0}' # Broadcast Nightlight
      
    - name: "WLED 2"
      unique_id: "wled_2_button"
      command_topic: "espnow/outgoing"
      payload_press: '{"device_platform": "wiz_remote", "button": 17, "channel": 0}' # Broadcast Nightlight
      
    - name: "WLED 3"
      unique_id: "wled_3_button"
      command_topic: "espnow/outgoing"
      payload_press: '{"device_platform": "wiz_remote", "button": 18, "channel": 0}' # Broadcast Nightlight
      
    - name: "WLED 4"
      unique_id: "wled_4_button"
      command_topic: "espnow/outgoing"
      payload_press: '{"device_platform": "wiz_remote", "button": 19, "channel": 0}' # Broadcast Nightlight

    - name: "WLED Brightness Up"
      unique_id: "wled_brightness_up_button"
      command_topic: "espnow/outgoing"
      payload_press: '{"device_platform": "wiz_remote", "button": 9, "channel": 0}' # Broadcast Brightness Up

    - name: "WLED Brightness Down"
      unique_id: "wled_brightness_down_button"
      command_topic: "espnow/outgoing"
      payload_press: '{"device_platform": "wiz_remote", "button": 8, "channel": 0}' # Broadcast Brightness Down
```

## Additional Notes

-   **ESP32 MAC Address:** The MAC address used in the `espnow/incoming/<device_MAC>` topic refers to the ESP32 gateway's MAC address, not the MAC addresses of the WLED devices.
-   **WLED Compatibility:** Ensure your WLED devices have ESP-NOW receiver mode enabled and are running a compatible firmware version (0.13.0 or newer).
-   **Channel Usage:**
    *   If you specify a Wi-Fi channel (`channel` != 0), the ESP32 will temporarily switch to that channel to send the command and then reconnect to your main Wi-Fi network.
    *   Broadcasting (`channel` = 0) will send the message on all Wi-Fi channels.

## Troubleshooting Tips

-   **Check Serial Output:** Use the Arduino IDE Serial Monitor to observe debug messages and error logs from the ESP32 gateway.
-   **Monitor MQTT Broker:** Use an MQTT client to monitor messages on the relevant topics to verify that commands are being sent and received correctly.
-   **WLED Logs:** Check the logs on your WLED devices for any messages or errors related to ESP-NOW communication.

## Contributing

Feel free to contribute by opening issues or submitting pull requests to improve this project.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## TODO
- Add other device platforms
- Use QuickESPNow Library
- Allow easy hooking of other sensors
- Better handle non-json incoming messages
- Auto Discovery for HomeAssistant via MQTT
