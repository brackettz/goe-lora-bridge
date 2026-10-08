#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WiFi.h>

#include "shared.h"

namespace {

constexpr char GOE_BASE_URL[] = "http://192.168.4.1";
constexpr char STATUS_FILTER[] = "car,alw,amp,frc,nrg,eto,wh";
constexpr uint32_t POLL_INTERVAL_MS = 10000;
constexpr uint32_t HEARTBEAT_MS = 60000;
constexpr uint32_t WIFI_RETRY_MS = 30000;
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
uint32_t wifiAttemptMs = 0;
float lastRxRssi = 0;
float lastRxSnr = 0;
int16_t radioState = RADIOLIB_ERR_NONE;

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED || millis() - wifiAttemptMs < WIFI_RETRY_MS) return;
  wifiAttemptMs = millis();
  WiFi.disconnect();
  WiFi.begin(GOE_WIFI_SSID, GOE_WIFI_PASS);
}

bool httpGet(const String& path, String& body) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  http.setConnectTimeout(HTTP_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS);
  if (!http.begin(String(GOE_BASE_URL) + path)) return false;
  int code = http.GET();
  if (code == HTTP_CODE_OK) body = http.getString();
  http.end();
  return code == HTTP_CODE_OK;
}

uint16_t clampU16(double v) { return static_cast<uint16_t>(constrain(lround(v), 0L, 65535L)); }

bool pollCharger() {
  lastPollMs = millis();
  String body;
  JsonDocument doc;
  if (!httpGet(String("/api/status?filter=") + STATUS_FILTER, body) || deserializeJson(doc, body)) {
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
  return httpGet(String("/api/set?") + key + "=" + value, body);
}

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
  screen.wake(SCREEN_PACKET_WAKE_MS);

  if (hdr.counter < lastCmdCounter) return;
  if (hdr.counter > lastCmdCounter) {
    lastCmdCounter = hdr.counter;
    prefs.putUInt("lastCmd", lastCmdCounter);
    applyCommand(cmd);
  }
  ackPending = true;
}

void drawScreen() {
  bool wifiOk = WiFi.status() == WL_CONNECTED;
  bool chargerOk = state.flags & TLM_CHARGER_OK;
  String lines[StatusDisplay::LINES] = {
      "GARAGE  " + formatUptime(),
      "WiFi " + (wifiOk ? String(WiFi.RSSI()) + " dBm" : String("down")),
      radioState == RADIOLIB_ERR_NONE
          ? "go-e " + (chargerOk ? String(carStateName(state.car)) : String("unreachable"))
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

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(GOE_WIFI_SSID, GOE_WIFI_PASS);
  wifiAttemptMs = millis();
  lastPollMs = millis() - POLL_INTERVAL_MS;
}

void loop() {
  maintainWifi();
  handleRadio();

  if (millis() - lastPollMs >= POLL_INTERVAL_MS) pollCharger();
  if (radioState == RADIOLIB_ERR_NONE) maybeSendTelemetry();
  if (screen.update()) drawScreen();
}
