#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

#include <functional>

#include "shared.h"

namespace {

constexpr char DEVICE_ID[] = "goe_lora_bridge";
constexpr char TOPIC_STATE[] = "goe_lora/state";
constexpr char TOPIC_AVAILABILITY[] = "goe_lora/availability";
constexpr char TOPIC_BRIDGE[] = "goe_lora/bridge";
constexpr char TOPIC_SET_PREFIX[] = "goe_lora/set/";
constexpr char TOPIC_SET_AMP[] = "goe_lora/set/amp";
constexpr char TOPIC_SET_FRC[] = "goe_lora/set/frc";
constexpr char TOPIC_HA_STATUS[] = "homeassistant/status";

constexpr uint8_t AMP_MIN = 6;
constexpr uint8_t AMP_MAX = 16;  // 32 for 22 kW models
constexpr uint32_t LINK_STALE_MS = 180000;
constexpr uint32_t WIFI_RETRY_MS = 30000;
constexpr uint32_t MQTT_RETRY_MS = 5000;
constexpr uint32_t CMD_RETRY_MS = 30000;
constexpr uint8_t CMD_MAX_TRIES = 4;

Preferences prefs;
FrameCipher cipher;
FrameCounter txCounter;
LoraLink lora;
StatusDisplay screen;
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

Telemetry telemetry{};
bool haveTelemetry = false;
uint32_t lastTelemetryCounter = 0;
uint32_t lastRxMs = 0;
float lastRxRssi = 0;
float lastRxSnr = 0;
int8_t publishedAvailability = -1;

Command pendingCmd{};
uint8_t cmdFrame[MAX_FRAME_LEN];
size_t cmdFrameLen = 0;
uint32_t cmdCounter = 0;
uint8_t cmdTries = 0;
uint32_t cmdNextAttemptMs = 0;

uint32_t wifiAttemptMs = 0;
uint32_t mqttAttemptMs = 0;
int16_t radioState = RADIOLIB_ERR_NONE;

bool cmdPending() { return pendingCmd.mask != 0; }

void queueCommand(uint8_t mask, uint8_t amp, uint8_t frc) {
  pendingCmd.mask |= mask;
  if (mask & CMD_SET_AMP) pendingCmd.amp = amp;
  if (mask & CMD_SET_FRC) pendingCmd.frc = frc;
  cmdCounter = txCounter.next();
  cmdFrameLen = cipher.seal(NODE_HOME_ID, MSG_COMMAND, cmdCounter, &pendingCmd, sizeof pendingCmd, cmdFrame);
  cmdTries = 0;
  cmdNextAttemptMs = millis();
}

void maybeSendCommand() {
  if (!cmdPending() || cmdFrameLen == 0) return;
  if (static_cast<int32_t>(millis() - cmdNextAttemptMs) < 0 || !lora.canTransmit()) return;
  if (cmdTries >= CMD_MAX_TRIES) {
    pendingCmd = {};
    return;
  }
  // Retries resend the identical frame, so the garage acks without re-applying it.
  if (lora.transmit(cmdFrame, cmdFrameLen)) {
    cmdTries++;
    cmdNextAttemptMs = millis() + CMD_RETRY_MS;
  }
}

bool linkFresh() { return haveTelemetry && millis() - lastRxMs < LINK_STALE_MS; }

void publishAvailability(bool force) {
  int8_t online = linkFresh() && (telemetry.flags & TLM_CHARGER_OK) ? 1 : 0;
  if (!mqtt.connected() || (!force && online == publishedAvailability)) return;
  if (mqtt.publish(TOPIC_AVAILABILITY, online ? "online" : "offline", true)) publishedAvailability = online;
}

void publishState() {
  if (!mqtt.connected() || !haveTelemetry) return;
  JsonDocument doc;
  doc["car"] = carStateName(telemetry.car);
  doc["allowed"] = telemetry.flags & TLM_ALLOWED ? "ON" : "OFF";
  doc["frc"] = frcName(telemetry.frc);
  doc["amp"] = telemetry.amp;
  doc["power"] = telemetry.powerW;
  for (uint8_t i = 0; i < 3; i++) {
    doc["voltage_l" + String(i + 1)] = telemetry.voltageV[i];
    doc["current_l" + String(i + 1)] = telemetry.currentCa[i] / 100.0;
  }
  doc["energy_total"] = telemetry.energyTotalWh / 1000.0;
  doc["energy_session"] = telemetry.energySessionWh / 1000.0;
  doc["rssi"] = lroundf(lastRxRssi);
  doc["snr"] = lastRxSnr;
  doc["garage_rssi"] = telemetry.cmdRssi;
  doc["garage_snr"] = telemetry.cmdSnrQ / 4.0;

  char buf[512];
  size_t len = serializeJson(doc, buf);
  mqtt.publish(TOPIC_STATE, reinterpret_cast<const uint8_t*>(buf), len, true);
}

void publishEntity(const char* component, const char* key, const char* name, bool needsCharger,
                   const std::function<void(JsonDocument&)>& extra = nullptr) {
  JsonDocument doc;
  String uid = String(DEVICE_ID) + "_" + key;
  doc["name"] = name;
  doc["unique_id"] = uid;
  doc["object_id"] = uid;
  doc["state_topic"] = TOPIC_STATE;
  doc["value_template"] = String("{{ value_json.") + key + " }}";

  JsonArray avty = doc["availability"].to<JsonArray>();
  avty.add<JsonObject>()["topic"] = TOPIC_BRIDGE;
  if (needsCharger) avty.add<JsonObject>()["topic"] = TOPIC_AVAILABILITY;
  doc["availability_mode"] = "all";

  JsonObject dev = doc["device"].to<JsonObject>();
  dev["identifiers"].to<JsonArray>().add(DEVICE_ID);
  dev["name"] = "go-e Charger";
  dev["manufacturer"] = "go-e";
  dev["model"] = "go-eCharger via LoRa bridge";

  if (extra) extra(doc);

  String topic = String("homeassistant/") + component + "/" + DEVICE_ID + "/" + key + "/config";
  String payload;
  serializeJson(doc, payload);
  mqtt.publish(topic.c_str(), reinterpret_cast<const uint8_t*>(payload.c_str()), payload.length(), true);
}

void publishDiscovery() {
  auto measurement = [](const char* deviceClass, const char* unit, int precision = -1) {
    return [=](JsonDocument& d) {
      d["device_class"] = deviceClass;
      d["unit_of_measurement"] = unit;
      d["state_class"] = "measurement";
      if (precision >= 0) d["suggested_display_precision"] = precision;
    };
  };
  auto diagnostic = [](const char* deviceClass, const char* unit) {
    return [=](JsonDocument& d) {
      if (deviceClass) d["device_class"] = deviceClass;
      d["unit_of_measurement"] = unit;
      d["state_class"] = "measurement";
      d["entity_category"] = "diagnostic";
    };
  };
  auto energy = [](JsonDocument& d) {
    d["device_class"] = "energy";
    d["unit_of_measurement"] = "kWh";
    d["state_class"] = "total_increasing";
    d["suggested_display_precision"] = 2;
  };

  publishEntity("sensor", "power", "Power", true, measurement("power", "W", 0));
  publishEntity("sensor", "car", "Car state", true, [](JsonDocument& d) {
    d["device_class"] = "enum";
    JsonArray opts = d["options"].to<JsonArray>();
    for (uint8_t i = 0; i < 6; i++) opts.add(carStateName(i));
  });
  publishEntity("binary_sensor", "allowed", "Charging allowed", true);
  publishEntity("sensor", "energy_total", "Energy total", true, energy);
  publishEntity("sensor", "energy_session", "Energy session", true, energy);
  publishEntity("sensor", "voltage_l1", "Voltage L1", true, measurement("voltage", "V", 0));
  publishEntity("sensor", "voltage_l2", "Voltage L2", true, measurement("voltage", "V", 0));
  publishEntity("sensor", "voltage_l3", "Voltage L3", true, measurement("voltage", "V", 0));
  publishEntity("sensor", "current_l1", "Current L1", true, measurement("current", "A", 2));
  publishEntity("sensor", "current_l2", "Current L2", true, measurement("current", "A", 2));
  publishEntity("sensor", "current_l3", "Current L3", true, measurement("current", "A", 2));

  publishEntity("number", "amp", "Charging current", true, [](JsonDocument& d) {
    d["command_topic"] = TOPIC_SET_AMP;
    d["min"] = AMP_MIN;
    d["max"] = AMP_MAX;
    d["step"] = 1;
    d["mode"] = "box";
    d["unit_of_measurement"] = "A";
    d["device_class"] = "current";
    d["icon"] = "mdi:current-ac";
  });
  publishEntity("select", "frc", "Charge mode", true, [](JsonDocument& d) {
    d["command_topic"] = TOPIC_SET_FRC;
    JsonArray opts = d["options"].to<JsonArray>();
    for (uint8_t i = 0; i < FRC_COUNT; i++) opts.add(FRC_NAMES[i]);
    d["icon"] = "mdi:ev-station";
  });

  publishEntity("sensor", "rssi", "LoRa RSSI", false, diagnostic("signal_strength", "dBm"));
  publishEntity("sensor", "snr", "LoRa SNR", false, diagnostic(nullptr, "dB"));
  publishEntity("sensor", "garage_rssi", "LoRa RSSI garage", false, diagnostic("signal_strength", "dBm"));
  publishEntity("sensor", "garage_snr", "LoRa SNR garage", false, diagnostic(nullptr, "dB"));
}

void onMqttMessage(char* topic, uint8_t* payload, unsigned int len) {
  String value(reinterpret_cast<const char*>(payload), len);
  value.trim();

  if (strcmp(topic, TOPIC_HA_STATUS) == 0) {
    if (value == "online") {
      publishDiscovery();
      publishAvailability(true);
      publishState();
    }
  } else if (strcmp(topic, TOPIC_SET_AMP) == 0) {
    long amp = lroundf(value.toFloat());
    if (amp >= AMP_MIN && amp <= AMP_MAX) queueCommand(CMD_SET_AMP, amp, 0);
  } else if (strcmp(topic, TOPIC_SET_FRC) == 0) {
    for (uint8_t i = 0; i < FRC_COUNT; i++) {
      if (value.equalsIgnoreCase(FRC_NAMES[i])) queueCommand(CMD_SET_FRC, 0, i);
    }
  }
}

void maintainWifi() {
  if (WiFi.status() == WL_CONNECTED || millis() - wifiAttemptMs < WIFI_RETRY_MS) return;
  wifiAttemptMs = millis();
  WiFi.disconnect();
  WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASS);
}

// PubSubClient::connect() blocks for the TCP handshake; it is rate limited to keep the loop responsive.
void maintainMqtt() {
  if (mqtt.connected()) {
    mqtt.loop();
    return;
  }
  if (WiFi.status() != WL_CONNECTED || millis() - mqttAttemptMs < MQTT_RETRY_MS) return;
  mqttAttemptMs = millis();

  String clientId = String(DEVICE_ID) + "_" + String(static_cast<uint32_t>(ESP.getEfuseMac()), HEX);
  if (!mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS, TOPIC_BRIDGE, 1, true, "offline")) return;

  mqtt.publish(TOPIC_BRIDGE, "online", true);
  mqtt.subscribe((String(TOPIC_SET_PREFIX) + "#").c_str());
  mqtt.subscribe(TOPIC_HA_STATUS);
  publishDiscovery();
  publishAvailability(true);
  publishState();
}

void handleRadio() {
  RxPacket pkt;
  if (!lora.poll(pkt)) return;

  FrameHeader hdr;
  Telemetry rx;
  if (!FrameCipher::peek(pkt.data, pkt.len, hdr) || hdr.type != MSG_TELEMETRY || hdr.sender != NODE_GARAGE_ID) return;
  if (!cipher.open(pkt.data, pkt.len, hdr, &rx, sizeof rx)) return;
  if (haveTelemetry && hdr.counter <= lastTelemetryCounter) return;

  lastTelemetryCounter = hdr.counter;
  telemetry = rx;
  haveTelemetry = true;
  lastRxMs = millis();
  lastRxRssi = pkt.rssi;
  lastRxSnr = pkt.snr;
  screen.wake(SCREEN_PACKET_WAKE_MS);

  if (cmdPending() && telemetry.ackCounter >= cmdCounter) pendingCmd = {};

  publishState();
  publishAvailability(false);
}

void drawScreen() {
  bool wifiOk = WiFi.status() == WL_CONNECTED;
  String cmdLine = "CMD ";
  if (cmdPending()) {
    cmdLine += "#" + String(cmdCounter) + " try " + cmdTries + "/" + CMD_MAX_TRIES;
  } else {
    cmdLine += "idle  TX " + formatAgo(lora.lastTxMs());
  }
  String lines[StatusDisplay::LINES] = {
      "HOME  " + formatUptime(),
      "WiFi " + (wifiOk ? String(WiFi.RSSI()) + " dBm" : String("down")),
      radioState == RADIOLIB_ERR_NONE ? "MQTT " + String(mqtt.connected() ? "connected" : "down")
                                      : "LoRa init err " + String(radioState),
      "RX " + (lastRxMs ? String(lastRxRssi, 0) + " dBm " + String(lastRxSnr, 1) + " dB " + formatAgo(lastRxMs)
                        : String("-")),
      !haveTelemetry                        ? String("go-e -")
      : !(telemetry.flags & TLM_CHARGER_OK) ? String("go-e unreachable")
                                            : String(carStateName(telemetry.car)) + " " +
                                                  String(telemetry.powerW / 1000.0f, 2) + " kW " + telemetry.amp + " A",
      cmdLine,
  };
  screen.show(lines);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  screen.begin();

  prefs.begin("goe-lora", false);
  txCounter.begin(prefs, "txCtr");
  cipher.begin();

  radioState = lora.begin();
  if (radioState != RADIOLIB_ERR_NONE) Serial.printf("LoRa init failed: %d\n", radioState);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(HOME_WIFI_SSID, HOME_WIFI_PASS);
  wifiAttemptMs = millis();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(1024);
  mqtt.setSocketTimeout(3);
  mqtt.setCallback(onMqttMessage);
}

void loop() {
  maintainWifi();
  maintainMqtt();
  handleRadio();

  if (radioState == RADIOLIB_ERR_NONE) maybeSendCommand();
  publishAvailability(false);
  if (screen.update()) drawScreen();
}
