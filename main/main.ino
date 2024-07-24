#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Arduino.h>
#include <Bounce2.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>
#include <seesaw_neopixel.h>

/// Setup SPI connection between Teensy and MCP2518
static const byte MCP2517_SCK = 13;
static const byte MCP2517_SDI = 11;
static const byte MCP2517_SDO = 12;
static const byte MCP2517_CS = 10;
static const byte MCP2517_INT = 41;
// Why are SCK, SDI, SDO pins not passed to this object??
ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

/// Initialize Moteus object for ID 1
Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());
Moteus* moteus1_ptr = &moteus1;

/// ReplySender to Serial.Print
class SerialPrintReplySender {
 public:
  SerialPrintReplySender(Moteus** moteuses, size_t len) : len_{len} {
    moteuses_ = new Moteus*[len];
    memcpy(moteuses_, moteuses, len * sizeof(Moteus*));
  }
  ~SerialPrintReplySender() { delete[] moteuses_; }

  void snd() {
    Serial.print(F("time "));
    Serial.print(millis());

    for (size_t i = 0; i < len_; i++) {
      const auto& m = moteuses_[i];
      m->SetQuery();
      const auto& reply = m->last_result().values;
      Serial.print(F("  mode "));
      Serial.print(static_cast<int>(reply.mode));
      Serial.print(F("  pos "));
      Serial.print(reply.position);
      Serial.print(F("  vel "));
      Serial.print(reply.velocity);
      Serial.print(F("  fault "));
      Serial.print(reply.fault);
    }

    Serial.println();
  }

  Moteus** moteuses_;
  size_t len_;
  Metro metro_{500};
} rpl_sndr_serial_print{&moteus1_ptr, 1};

/// CommandReceiver from NeoKey1x4
class NeoKeyCommandReceiver {
 public:
  class NeoKeyDebouncer : public Debouncer {
   public:
    NeoKeyDebouncer() = default;

    NeoKeyDebouncer(Adafruit_NeoKey_1x4* neokey, const uint8_t key_idx,
                    const uint16_t interval_ms)
        : neokey_{neokey}, key_idx_{key_idx} {
      interval(interval_ms);
    }

    bool readCurrentState() override {
      return neokey_->read() & (1 << key_idx_);
    };

    Adafruit_NeoKey_1x4* neokey_;
    uint8_t key_idx_;
  } neokeys_[4];

  NeoKeyCommandReceiver(uint16_t interval_ms) {
    while (!neokey_.begin(0x30)) {
      Serial.println("Could not start NeoKey, check wiring?");
      delay(1000);
    }

    for (uint8_t i = 0; i < sizeof(neokeys_) / sizeof(neokeys_[0]); i++) {
      neokeys_[i] = NeoKeyDebouncer{&neokey_, i, interval_ms};
    }
  }

  void rcv() {
    if (!metro_.check()) return;

    for (size_t i = 0; i < sizeof(neokeys_) / sizeof(neokeys_[0]); i++) {
      neokeys_[i].update();
      if (neokeys_[i].rose()) {
        Serial.print("NeoKey #");
        Serial.print(i);
        Serial.println(" pressed");
        moteus1.SetPosition(
            Moteus::PositionMode::Command{.position = 0.25 * i});
      }
    }
  }

  Adafruit_NeoKey_1x4 neokey_;
  Metro metro_{100};
} cmd_rcvr_neokey{5};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    Serial.println(F("Serial problem"));
    delay(1000);
  }
  Serial.println(F("Serial started"));

  Wire.begin();
  Wire1.begin();
  delay(1000);
  SPI.begin();
  Serial.println(F("Wire, Wire1, SPI started"));

  /* Initialize moteuses */ {
    // This operates the CAN-FD bus at 1Mbit for both the arbitration and
    // data rate. Most arduino shields cannot operate at 5Mbps correctly, so the
    // moteus Arduino library permanently disables BRS.
    ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll,
                                DataBitRateFactor::x1);
    // The atmega32u4 on the CANbed has only a tiny amount of memory.
    // The ACAN2517FD driver needs custom settings so as to not exhaust
    // all of SRAM just with its buffers.
    settings.mArbitrationSJW = 2;
    settings.mDriverTransmitFIFOSize = 1;
    settings.mDriverReceiveFIFOSize = 2;
    const uint32_t errorCode = can.begin(settings, [] { can.isr(); });
    while (errorCode != 0) {
      Serial.print(F("CAN error 0x"));
      Serial.println(errorCode, HEX);
      delay(1000);
    }
    Serial.println(F("CAN FD started"));
  }

  moteus1.SetStop();
  Serial.println(F("All moteus stopped"));
}

void loop() {
  cmd_rcvr_neokey.rcv();
  rpl_sndr_serial_print.snd();
}