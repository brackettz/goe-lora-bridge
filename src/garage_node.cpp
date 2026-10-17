#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_netif.h>
#include <esp_wifi.h>

#include "shared.h"

namespace {

constexpr char STATUS_FILTER[] = "car,alw,amp,frc,nrg,eto,wh";
constexpr uint32_t POLL_INTERVAL_MS = 10000;
#ifdef DESK_TEST
constexpr uint32_t HEARTBEAT_MS = 15000;
#else
constexpr uint32_t HEARTBEAT_MS = 60000;
#endif
constexpr uint16_t HTTP_TIMEOUT_MS = 2000;
constexpr int32_t POWER_DELTA_W = 300;

Preferences prefs;
FrameCipher cipher;
FrameCounter txCounter;
LoraLink lora;
StatusDisplay screen;

Telemetry state{};
Telemetry lastSent{};
bool everSent = false;
bool ackPending = false;
uint32_t lastCmdCounter = 0;
uint32_t lastPollMs = 0;
uint32_t lastPollOkMs = 0;
uint32_t lastSendMs = 0;
uint32_t lastRxMs = 0;
IPAddress chargerIp;
float lastRxRssi = 0;
float lastRxSnr = 0;
int16_t radioState = RADIOLIB_ERR_NONE;
int pollError = 0;

#ifdef DESK_TEST
// Fake charger: a car charging on three phases at the configured current
bool pollCharger() {
  lastPollMs = millis();
  if (state.amp == 0) state.amp = 10;
  bool charging = state.frc != 1;
  state.flags = TLM_CHARGER_OK | (charging ? TLM_ALLOWED : 0);
  state.car = charging ? 2 : 4;
  for (uint8_t i = 0; i < 3; i++) {
    state.voltageV[i] = 228 + random(5);
    state.currentCa[i] = charging ? state.amp * 100 - random(30) : 0;
  }
  state.powerW = charging ? 3 * 230 * state.amp : 0;
  state.energySessionWh += state.powerW * (POLL_INTERVAL_MS / 1000) / 3600;
  state.energyTotalWh = 1234000 + state.energySessionWh;
  lastPollOkMs = millis();
  return true;
}

bool setChargerValue(const char* key, int value) {
  if (strcmp(key, "amp") == 0) state.amp = value;
  if (strcmp(key, "frc") == 0) state.frc = value;
  return true;
}
#else
bool httpGet(IPAddress ip, const String& path, String& body) {
  HTTPClient http;
  http.setConnectTimeout(HTTP_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin("http://" + ip.toString() + path)) return false;
  int code = http.GET();
  if (code == HTTP_CODE_OK) body = http.getString();
  http.end();
  pollError = code == HTTP_CODE_OK ? 0 : code;
  return code == HTTP_CODE_OK;
}

// The charger joins our access point as a client. Other clients (e.g. a phone) may be connected too,
// so every leased address is probed until one answers the status request.
bool fetchStatus(const String& path, String& body) {
  if (chargerIp && httpGet(chargerIp, path, body)) return true;
  chargerIp = IPAddress();
  pollError = 0;

  wifi_sta_list_t wifiList;
  esp_netif_sta_list_t netifList;
  if (esp_wifi_ap_get_sta_list(&wifiList) != ESP_OK || esp_netif_get_sta_list(&wifiList, &netifList) != ESP_OK) {
    return false;
  }
  for (int i = 0; i < netifList.num; i++) {
    IPAddress ip(netifList.sta[i].ip.addr);
    if (ip && httpGet(ip, path, body)) {
      chargerIp = ip;
      return true;
    }
  }
  return false;
}

uint16_t clampU16(double v) { return static_cast<uint16_t>(constrain(lround(v), 0L, 65535L)); }

bool pollCharger() {
  lastPollMs = millis();
  String body;
  JsonDocument doc;
  bool ok = fetchStatus(String("/api/status?filter=") + STATUS_FILTER, body);
  if (ok && deserializeJson(doc, body)) {
    ok = false;
    pollError = 1;
  }
  if (!ok) {
    Serial.printf("go-e poll failed: %d\n", pollError);
    state.flags &= ~TLM_CHARGER_OK;
    return false;
  }

  state.flags = TLM_CHARGER_OK | (doc["alw"].as<bool>() ? TLM_ALLOWED : 0);
  state.car = doc["car"] | 0;
  state.frc = doc["frc"] | 0;
  state.amp = doc["amp"] | 0;

  // nrg: U L1..L3, U N, I L1..L3, P L1..L3, P N, P total, pf L1..L3, pf N
  JsonArrayConst nrg = doc["nrg"];
  for (uint8_t i = 0; i < 3; i++) {
    state.voltageV[i] = clampU16(nrg[i] | 0.0);
    state.currentCa[i] = clampU16((nrg[4 + i] | 0.0) * 100.0);
  }
  state.powerW = clampU16(nrg[11] | 0.0);
  state.energyTotalWh = static_cast<uint32_t>(doc["eto"] | 0ULL);
  state.energySessionWh = static_cast<uint32_t>(lround(doc["wh"] | 0.0));
  lastPollOkMs = millis();
  return true;
}

bool setChargerValue(const char* key, int value) {
  String body;
  return chargerIp && httpGet(chargerIp, String("/api/set?") + key + "=" + value, body);
}
#endif

bool telemetryChanged() {
  if (!everSent) return true;
  if (state.flags != lastSent.flags || state.car != lastSent.car || state.frc != lastSent.frc ||
      state.amp != lastSent.amp) {
    return true;
  }
  return abs(static_cast<int32_t>(state.powerW) - static_cast<int32_t>(lastSent.powerW)) >= POWER_DELTA_W;
}

void maybeSendTelemetry() {
  if (!lora.canTransmit()) return;
  if (!ackPending && !telemetryChanged() && millis() - lastSendMs < HEARTBEAT_MS) return;

  state.ackCounter = lastCmdCounter;
  state.cmdRssi = static_cast<int8_t>(constrain(lroundf(lastRxRssi), -128L, 127L));
  state.cmdSnrQ = static_cast<int8_t>(constrain(lroundf(lastRxSnr * 4), -128L, 127L));

  uint8_t frame[MAX_FRAME_LEN];
  size_t len = cipher.seal(NODE_GARAGE_ID, MSG_TELEMETRY, txCounter.next(), &state, sizeof state, frame);
  if (len == 0 || !lora.transmit(frame, len)) return;

  lastSent = state;
  everSent = true;
  ackPending = false;
  lastSendMs = millis();
}

void applyCommand(const Command& cmd) {
  if (cmd.mask & CMD_SET_AMP) setChargerValue("amp", cmd.amp);
  if (cmd.mask & CMD_SET_FRC && cmd.frc < FRC_COUNT) setChargerValue("frc", cmd.frc);
  pollCharger();
}

void handleRadio() {
  RxPacket pkt;
  if (!lora.poll(pkt)) return;

  FrameHeader hdr;
  Command cmd;
  if (!FrameCipher::peek(pkt.data, pkt.len, hdr) || hdr.type != MSG_COMMAND || hdr.sender != NODE_HOME_ID) return;
  if (!cipher.open(pkt.data, pkt.len, hdr, &cmd, sizeof cmd)) return;

  lastRxMs = millis();
  lastRxRssi = pkt.rssi;
  lastRxSnr = pkt.snr;

  if (hdr.counter < lastCmdCounter) return;
  if (hdr.counter > lastCmdCounter) {
    lastCmdCounter = hdr.counter;
    prefs.putUInt("lastCmd", lastCmdCounter);
    applyCommand(cmd);
  }
  ackPending = true;
}

void drawScreen() {
  bool chargerOk = state.flags & TLM_CHARGER_OK;
  String lines[StatusDisplay::LINES] = {
      "GARAGE  " + formatUptime(),
#ifdef DESK_TEST
      String("DESK TEST ") + lora.femName(),
#else
      "AP " + String(WiFi.softAPgetStationNum()) + " client " + (chargerIp ? chargerIp.toString() : String("-")),
#endif
      radioState == RADIOLIB_ERR_NONE
          ? "go-e " + (chargerOk                ? String(carStateName(state.car))
                       : pollError == 1         ? String("bad JSON")
                       : pollError              ? "HTTP " + String(pollError)
                                                : String("not connected"))
          : "LoRa init err " + String(radioState),
      String(state.powerW / 1000.0f, 2) + " kW  " + state.amp + " A  " + frcName(state.frc),
      "TX #" + String(txCounter.value()) + "  " + formatAgo(lora.lastTxMs()) +
          (lora.txActive() ? " ..." : ""),
      "RX " + (lastRxMs ? String(lastRxRssi, 0) + " dBm " + String(lastRxSnr, 1) + " dB " + formatAgo(lastRxMs)
                        : String("-")),
  };
  screen.show(lines);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  screen.begin();

  prefs.begin("goe-lora", false);
  lastCmdCounter = prefs.getUInt("lastCmd", 0);
  txCounter.begin(prefs, "txCtr");
  cipher.begin();

  radioState = lora.begin();
  if (radioState != RADIOLIB_ERR_NONE) Serial.printf("LoRa init failed: %d\n", radioState);

#ifndef DESK_TEST
  WiFi.mode(WIFI_AP);
  WiFi.softAP(GOE_AP_SSID, GOE_AP_PASS);
#endif
  lastPollMs = millis() - POLL_INTERVAL_MS;
}

void loop() {
  handleRadio();

  if (millis() - lastPollMs >= POLL_INTERVAL_MS) pollCharger();
  if (radioState == RADIOLIB_ERR_NONE) maybeSendTelemetry();
  if (screen.update()) drawScreen();
}
