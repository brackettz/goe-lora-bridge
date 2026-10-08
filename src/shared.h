#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>
#include <SSD1306Wire.h>
#include <mbedtls/gcm.h>

#include "secrets.h"

namespace pins {
constexpr uint8_t LORA_NSS = 8;
constexpr uint8_t LORA_SCK = 9;
constexpr uint8_t LORA_MOSI = 10;
constexpr uint8_t LORA_MISO = 11;
constexpr uint8_t LORA_RST = 12;
constexpr uint8_t LORA_BUSY = 13;
constexpr uint8_t LORA_DIO1 = 14;
// GC1109 front end (Heltec V4 only)
constexpr uint8_t PA_POWER = 7;
constexpr uint8_t PA_EN = 2;
constexpr uint8_t PA_TX_EN = 46;
constexpr uint8_t OLED_SDA = 17;
constexpr uint8_t OLED_SCL = 18;
constexpr uint8_t OLED_RST = 21;
constexpr uint8_t VEXT = 36;
constexpr uint8_t BUTTON = 0;
}  // namespace pins

namespace radio_cfg {
// EU868 sub-band 869.40-869.65 MHz: 500 mW ERP, 10 % duty cycle
constexpr float FREQ_MHZ = 869.525f;
constexpr float BW_KHZ = 125.0f;
constexpr uint8_t SF = 11;
constexpr uint8_t CR = 5;
constexpr uint8_t SYNC_WORD = 0x12;
// SX1262 output before the GC1109 PA; keep total radiated power <= 27 dBm ERP
constexpr int8_t TX_POWER_DBM = 10;
constexpr uint16_t PREAMBLE_LEN = 12;
constexpr float TCXO_VOLTAGE = 1.8f;
constexpr uint32_t DUTY_CYCLE_PERCENT = 10;
constexpr uint32_t TX_TIMEOUT_MS = 10000;
}  // namespace radio_cfg

constexpr uint8_t NODE_GARAGE_ID = 1;
constexpr uint8_t NODE_HOME_ID = 2;

constexpr uint32_t SCREEN_TIMEOUT_MS = 60000;
constexpr uint32_t SCREEN_PACKET_WAKE_MS = 10000;
constexpr uint32_t SCREEN_REFRESH_MS = 500;

enum MsgType : uint8_t { MSG_TELEMETRY = 1, MSG_COMMAND = 2 };

constexpr uint8_t FRAME_MAGIC = 0x6E;

struct __attribute__((packed)) FrameHeader {
  uint8_t magic;
  uint8_t sender;
  uint8_t type;
  uint32_t counter;
};

constexpr uint8_t TLM_CHARGER_OK = 0x01;
constexpr uint8_t TLM_ALLOWED = 0x02;

struct __attribute__((packed)) Telemetry {
  uint32_t ackCounter;
  uint8_t flags;
  uint8_t car;
  uint8_t frc;
  uint8_t amp;
  uint16_t voltageV[3];
  uint16_t currentCa[3];
  uint16_t powerW;
  uint32_t energyTotalWh;
  uint32_t energySessionWh;
  int8_t cmdRssi;
  int8_t cmdSnrQ;  // SNR * 4
};

constexpr uint8_t CMD_SET_AMP = 0x01;
constexpr uint8_t CMD_SET_FRC = 0x02;

struct __attribute__((packed)) Command {
  uint8_t mask;
  uint8_t amp;
  uint8_t frc;
};

constexpr size_t GCM_TAG_LEN = 16;
constexpr size_t MAX_FRAME_LEN = sizeof(FrameHeader) + 64 + GCM_TAG_LEN;

inline const char* carStateName(uint8_t car) {
  static const char* const names[] = {"Unknown", "Idle", "Charging", "Waiting for car", "Complete", "Error"};
  return car < 6 ? names[car] : names[0];
}

inline const char* const FRC_NAMES[] = {"Neutral", "Off", "On"};
constexpr uint8_t FRC_COUNT = 3;

inline const char* frcName(uint8_t frc) { return frc < FRC_COUNT ? FRC_NAMES[frc] : FRC_NAMES[0]; }

// AES-256-GCM: header is authenticated as AAD, nonce = sender id + counter.
class FrameCipher {
 public:
  void begin() {
    static const uint8_t key[32] = LORA_AES_KEY;
    mbedtls_gcm_init(&ctx_);
    mbedtls_gcm_setkey(&ctx_, MBEDTLS_CIPHER_ID_AES, key, 256);
  }

  size_t seal(uint8_t sender, MsgType type, uint32_t counter, const void* plain, size_t len, uint8_t* out) {
    if (sizeof(FrameHeader) + len + GCM_TAG_LEN > MAX_FRAME_LEN) return 0;
    FrameHeader hdr{FRAME_MAGIC, sender, type, counter};
    memcpy(out, &hdr, sizeof hdr);
    uint8_t nonce[12];
    makeNonce(hdr, nonce);
    uint8_t* cipherText = out + sizeof hdr;
    int rc = mbedtls_gcm_crypt_and_tag(&ctx_, MBEDTLS_GCM_ENCRYPT, len, nonce, sizeof nonce, out, sizeof hdr,
                                       static_cast<const uint8_t*>(plain), cipherText, GCM_TAG_LEN, cipherText + len);
    return rc == 0 ? sizeof hdr + len + GCM_TAG_LEN : 0;
  }

  static bool peek(const uint8_t* in, size_t len, FrameHeader& hdr) {
    if (len < sizeof hdr + GCM_TAG_LEN) return false;
    memcpy(&hdr, in, sizeof hdr);
    return hdr.magic == FRAME_MAGIC;
  }

  bool open(const uint8_t* in, size_t len, FrameHeader& hdr, void* plain, size_t plainLen) {
    if (len != sizeof hdr + plainLen + GCM_TAG_LEN || !peek(in, len, hdr)) return false;
    uint8_t nonce[12];
    makeNonce(hdr, nonce);
    const uint8_t* cipherText = in + sizeof hdr;
    return mbedtls_gcm_auth_decrypt(&ctx_, plainLen, nonce, sizeof nonce, in, sizeof hdr, cipherText + plainLen,
                                    GCM_TAG_LEN, cipherText, static_cast<uint8_t*>(plain)) == 0;
  }

 private:
  static void makeNonce(const FrameHeader& hdr, uint8_t* nonce) {
    memset(nonce, 0, 12);
    nonce[0] = hdr.sender;
    memcpy(nonce + 8, &hdr.counter, sizeof hdr.counter);
  }

  mbedtls_gcm_context ctx_;
};

// Monotonic TX counter that survives reboots. NVS is only written once per block
// so a GCM nonce is never reused and the receiver's replay check keeps working.
class FrameCounter {
 public:
  void begin(Preferences& prefs, const char* key) {
    prefs_ = &prefs;
    key_ = key;
    value_ = prefs.getUInt(key, 0);
    reserve();
  }

  uint32_t next() {
    if (++value_ >= reserved_) reserve();
    return value_;
  }

  uint32_t value() const { return value_; }

 private:
  static constexpr uint32_t BLOCK = 1000;

  void reserve() {
    reserved_ = value_ + BLOCK;
    prefs_->putUInt(key_, reserved_);
  }

  Preferences* prefs_ = nullptr;
  const char* key_ = nullptr;
  uint32_t value_ = 0;
  uint32_t reserved_ = 0;
};

struct RxPacket {
  uint8_t data[MAX_FRAME_LEN];
  size_t len;
  float rssi;
  float snr;
};

namespace radio_irq {
static volatile bool pending = false;
static void IRAM_ATTR onDio1() { pending = true; }
}  // namespace radio_irq

// Interrupt-driven half-duplex link. Always returns to continuous RX after a
// transmission and enforces the duty cycle from the measured airtime.
class LoraLink {
 public:
  LoraLink()
      : spi_(FSPI), radio_(new Module(pins::LORA_NSS, pins::LORA_DIO1, pins::LORA_RST, pins::LORA_BUSY, spi_)) {}

  int16_t begin() {
    pinMode(pins::PA_POWER, OUTPUT);
    pinMode(pins::PA_EN, OUTPUT);
    pinMode(pins::PA_TX_EN, OUTPUT);
    digitalWrite(pins::PA_POWER, HIGH);
    digitalWrite(pins::PA_EN, HIGH);
    digitalWrite(pins::PA_TX_EN, LOW);

    spi_.begin(pins::LORA_SCK, pins::LORA_MISO, pins::LORA_MOSI, pins::LORA_NSS);
    int16_t state = radio_.begin(radio_cfg::FREQ_MHZ, radio_cfg::BW_KHZ, radio_cfg::SF, radio_cfg::CR,
                                 radio_cfg::SYNC_WORD, radio_cfg::TX_POWER_DBM, radio_cfg::PREAMBLE_LEN,
                                 radio_cfg::TCXO_VOLTAGE, false);
    if (state != RADIOLIB_ERR_NONE) return state;
    radio_.setDio2AsRfSwitch(true);
    radio_.setCurrentLimit(140);
    radio_.setRxBoostedGainMode(true);
    radio_.setDio1Action(radio_irq::onDio1);
    return startRx();
  }

  bool canTransmit() const { return !txActive_ && static_cast<int32_t>(millis() - nextTxAllowedMs_) >= 0; }

  bool transmit(const uint8_t* data, size_t len) {
    if (!canTransmit()) return false;
    digitalWrite(pins::PA_TX_EN, HIGH);
    radio_irq::pending = false;
    if (radio_.startTransmit(data, len) != RADIOLIB_ERR_NONE) {
      digitalWrite(pins::PA_TX_EN, LOW);
      startRx();
      return false;
    }
    txActive_ = true;
    txStartMs_ = millis();
    return true;
  }

  bool poll(RxPacket& pkt) {
    if (txActive_ && millis() - txStartMs_ > radio_cfg::TX_TIMEOUT_MS) {
      radio_irq::pending = false;
      finishTx();
      return false;
    }
    if (!radio_irq::pending) return false;
    radio_irq::pending = false;

    if (txActive_) {
      finishTx();
      return false;
    }

    size_t len = radio_.getPacketLength();
    bool ok = len > 0 && len <= sizeof pkt.data && radio_.readData(pkt.data, len) == RADIOLIB_ERR_NONE;
    if (ok) {
      pkt.len = len;
      pkt.rssi = radio_.getRSSI();
      pkt.snr = radio_.getSNR();
    }
    startRx();
    return ok;
  }

  bool txActive() const { return txActive_; }
  uint32_t lastTxMs() const { return lastTxMs_; }
  uint32_t lastAirtimeMs() const { return lastAirtimeMs_; }

 private:
  int16_t startRx() { return radio_.startReceive(); }

  void finishTx() {
    radio_.finishTransmit();
    digitalWrite(pins::PA_TX_EN, LOW);
    txActive_ = false;
    lastTxMs_ = millis();
    lastAirtimeMs_ = lastTxMs_ - txStartMs_;
    nextTxAllowedMs_ = lastTxMs_ + lastAirtimeMs_ * (100 / radio_cfg::DUTY_CYCLE_PERCENT - 1);
    startRx();
  }

  SPIClass spi_;
  SX1262 radio_;
  bool txActive_ = false;
  uint32_t txStartMs_ = 0;
  uint32_t lastTxMs_ = 0;
  uint32_t lastAirtimeMs_ = 0;
  uint32_t nextTxAllowedMs_ = 0;
};

inline String formatAgo(uint32_t sinceMs) {
  if (sinceMs == 0) return "-";
  uint32_t s = (millis() - sinceMs) / 1000;
  if (s < 120) return String(s) + "s";
  if (s < 7200) return String(s / 60) + "m";
  return String(s / 3600) + "h";
}

inline String formatUptime() {
  uint32_t s = millis() / 1000;
  char buf[16];
  snprintf(buf, sizeof buf, "%02lu:%02lu:%02lu", static_cast<unsigned long>(s / 3600),
           static_cast<unsigned long>(s / 60 % 60), static_cast<unsigned long>(s % 60));
  return buf;
}

class StatusDisplay {
 public:
  static constexpr uint8_t LINES = 6;

  StatusDisplay() : oled_(0x3c, pins::OLED_SDA, pins::OLED_SCL) {}

  // Blocking waits are limited to the one-time power-up/reset sequence.
  void begin() {
    pinMode(pins::BUTTON, INPUT_PULLUP);
    pinMode(pins::VEXT, OUTPUT);
    digitalWrite(pins::VEXT, LOW);
    delay(50);
    pinMode(pins::OLED_RST, OUTPUT);
    digitalWrite(pins::OLED_RST, LOW);
    delay(20);
    digitalWrite(pins::OLED_RST, HIGH);
    delay(20);
    oled_.init();
    oled_.flipScreenVertically();
    oled_.setFont(ArialMT_Plain_10);
    oled_.setTextAlignment(TEXT_ALIGN_LEFT);
    on_ = true;
    wake(SCREEN_TIMEOUT_MS);
  }

  void wake(uint32_t durationMs) {
    uint32_t until = millis() + durationMs;
    if (!on_) {
      oled_.displayOn();
      on_ = true;
      offAtMs_ = until;
    } else if (static_cast<int32_t>(until - offAtMs_) > 0) {
      offAtMs_ = until;
    }
    lastDrawMs_ = millis() - SCREEN_REFRESH_MS;
  }

  // Returns true when the caller should render a new frame.
  bool update() {
    if (digitalRead(pins::BUTTON) == LOW) wake(SCREEN_TIMEOUT_MS);
    if (!on_) return false;
    if (static_cast<int32_t>(millis() - offAtMs_) >= 0) {
      oled_.clear();
      oled_.display();
      oled_.displayOff();
      on_ = false;
      return false;
    }
    if (millis() - lastDrawMs_ < SCREEN_REFRESH_MS) return false;
    lastDrawMs_ = millis();
    return true;
  }

  void show(const String (&lines)[LINES]) {
    oled_.clear();
    for (uint8_t i = 0; i < LINES; i++) oled_.drawString(0, i * 10, lines[i]);
    oled_.display();
  }

 private:
  SSD1306Wire oled_;
  bool on_ = false;
  uint32_t offAtMs_ = 0;
  uint32_t lastDrawMs_ = 0;
};
