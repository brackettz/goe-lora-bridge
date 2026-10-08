#pragma once

// Copy to include/secrets.h and fill in. Generate a key with:
//   python3 -c "import os;print(', '.join(f'0x{b:02x}' for b in os.urandom(32)))"
#define LORA_AES_KEY {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define GOE_WIFI_SSID "go-eCharger_000000"
#define GOE_WIFI_PASS "changeme"

#define HOME_WIFI_SSID "MyWifi"
#define HOME_WIFI_PASS "changeme"

#define MQTT_HOST "192.168.1.10"
#define MQTT_PORT 1883
#define MQTT_USER "goe"
#define MQTT_PASS "changeme"
