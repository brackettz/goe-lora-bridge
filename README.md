# goe-lora-bridge

Bridge a [go-e Charger](https://go-e.com) in an underground car park without internet or Wi-Fi coverage to
Home Assistant, using an encrypted 868 MHz LoRa point-to-point link between two ESP32-S3 boards.

```
 Garage                                               Apartment
┌────────────┐  Wi-Fi   ┌─────────────┐   LoRa    ┌─────────────┐  Wi-Fi/MQTT  ┌────────────────┐
│ go-e       │◄────────►│ garage_node │◄─────────►│ home_node   │◄────────────►│ Mosquitto + HA │
│ Charger    │ HTTP API │ (Heltec V4) │ 869.5 MHz │ (Heltec V4) │              └────────────────┘
└────────────┘   v2     └─────────────┘  AES-GCM  └─────────────┘
```

## Features

- Polls the charger every 10 s via the local [HTTP API v2](https://github.com/goecharger/go-eCharger-API-v2)
- Controls charging current (`amp`) and charge mode (`frc`: Neutral / Off / On) from Home Assistant
- AES-256-GCM encrypted and authenticated frames with replay protection
- Home Assistant MQTT auto-discovery: the charger shows up as a device
- Command acknowledgement with automatic retries
- Duty cycle limiting built into the firmware
- OLED status screen on both nodes, with a screen saver

## Hardware

| Part | Purpose |
| --- | --- |
| 2× Heltec WiFi LoRa 32 V4 (ESP32-S3, SX1262, 0.96" SSD1306 OLED) | `garage_node` and `home_node` |
| Mean Well HDR-15-5 (5 V DIN rail PSU) | Powers the garage node |
| 868 MHz antennas | Use the best antenna you can mount; reinforced concrete is the main obstacle |

> ⚠️ Mains wiring of the power supply must be done by a qualified electrician.

## Getting started

1. Install [PlatformIO](https://platformio.org/).
2. Create your secrets file:
   ```sh
   cp include/secrets.example.h include/secrets.h
   ```
3. Generate a random key and paste it into `LORA_AES_KEY`. Both nodes must use the same key.
   ```sh
   python3 -c "import os;print(', '.join(f'0x{b:02x}' for b in os.urandom(32)))"
   ```
4. Fill in the charger hotspot credentials (`GOE_WIFI_*`, printed on the charger's reset card), your home Wi-Fi
   and your MQTT broker.
5. In the go-e app, enable **HTTP API v2** and keep the charger's Wi-Fi hotspot enabled.
6. Flash both boards:
   ```sh
   pio run -e garage_node -t upload
   pio run -e home_node -t upload
   ```

`include/secrets.h` is git-ignored. Never commit it, and don't publish built firmware images: the secrets are
compiled into them.

## Configuration

Radio and timing constants live in `src/shared.h` and at the top of each node's source file.

| Setting | Default | Notes |
| --- | --- | --- |
| `FREQ_MHZ` | 869.525 | EU sub-band 869.40–869.65 MHz: 500 mW ERP, 10 % duty cycle |
| `SF` / `BW_KHZ` / `CR` | 11 / 125 / 4:5 | ≈ 1.3 s airtime per telemetry frame |
| `TX_POWER_DBM` | 10 | SX1262 output *before* the V4's GC1109 PA; stay within the legal ERP limit |
| `POLL_INTERVAL_MS` | 10 s | Charger polling interval |
| `HEARTBEAT_MS` | 60 s | Telemetry is sent on change, otherwise at this interval |
| `POWER_DELTA_W` | 300 W | Power change that triggers an immediate telemetry frame |
| `AMP_MAX` | 16 A | Set to 32 for 22 kW chargers |

Both nodes must use identical radio settings. If you change the frequency or the spreading factor, check that you
still comply with your local regulations.

## Home Assistant

The device appears automatically once the home node is connected to MQTT and the first telemetry frame arrives.

| Entity | Type |
| --- | --- |
| Power, Voltage L1–L3, Current L1–L3 | sensor |
| Energy total, Energy session | sensor (energy, usable in the Energy dashboard) |
| Car state | sensor (enum) |
| Charging allowed | binary_sensor |
| Charging current | number (6 A – `AMP_MAX`) |
| Charge mode | select (Neutral / Off / On) |
| LoRa RSSI / SNR, garage and home side | diagnostic sensors |

### MQTT topics

| Topic | Content |
| --- | --- |
| `goe_lora/state` | JSON state (retained) |
| `goe_lora/availability` | `online` while the link is fresh and the charger is reachable |
| `goe_lora/bridge` | Home node status (`online` / `offline` via LWT) |
| `goe_lora/set/amp` | Command: charging current in A |
| `goe_lora/set/frc` | Command: `Neutral`, `Off` or `On` |

Entities go unavailable if no telemetry has been received for 3 minutes.

## How it works

**Polling and sending.** The garage node polls the charger every 10 s but only transmits when something
relevant changes or the heartbeat is due. That keeps the transmitter within the 10 % duty cycle at high
spreading factors. Each node waits 9× the measured airtime after a transmission before sending again.

**Commands.** Home Assistant commands are queued on the home node and sent to the garage. The garage applies them
through `/api/set` and acknowledges them in its next telemetry frame. Unacknowledged commands are resent up to
4 times, 30 s apart.

**Security.** Frames are encrypted with AES-256-GCM. The unencrypted frame header (sender, type, counter) is
authenticated as additional data. Each node keeps a monotonic frame counter in NVS, which serves as the GCM nonce
and as replay protection. If you fully erase the flash of one node (`erase_flash`), erase both. Otherwise the
other node rejects its frames as replays.

## Display

The first line shows the node name and uptime. The other lines show:

- Wi-Fi signal strength
- MQTT status (home node) or charger status (garage node)
- RSSI and SNR of the last received frame
- Charger power and current, plus the TX or command state

The display turns off after 60 s. It wakes for 10 s when a frame arrives, and for 60 s when you press the PRG button.

## Status

The firmware compiles but has not been tested on real hardware yet. These Heltec V4 specifics still need to be
verified:

- GC1109 PA control pins (7 / 2 / 46) and their polarity
- Effective output power with the PA
- Native USB CDC serial output (`ARDUINO_USB_CDC_ON_BOOT`)
