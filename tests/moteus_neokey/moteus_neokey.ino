// Drive moteus of ID 1 with a NeoKey1x4 connected to the I2C0 port.
// Button #i commands moteus to position 0.25 * i.

#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>

namespace mm = mjbots::moteus;
using PmCmd = mm::PositionMode::Command;
using PmFmt = mm::PositionMode::Format;
using QRpl = mm::Query::Result;
using QFmt = mm::Query::Format;
using Res = mm::Resolution;

#define MCP2518FD_CS 10
#define MCP2518FD_INT 41
ACAN2517FD canfd_driver(MCP2518FD_CS, SPI, MCP2518FD_INT);

Moteus mot1(canfd_driver, []() {
  Moteus::Options options;
  options.id = 1;
  options.query_format = [] {
    QFmt fmt;
    fmt.abs_position = Res::kFloat;
    fmt.motor_temperature = Res::kInt16;
    fmt.trajectory_complete = Res::kInt8;
    return fmt;
  }();
  return options;
}());

PmFmt pm_fmt{.velocity = Res::kIgnore,
             .maximum_torque = Res::kFloat,
             .velocity_limit = Res::kFloat,
             .accel_limit = Res::kFloat};

class SerialPrintReplySender {
 public:
  void snd() {
    if (!metro_.check()) return;

    Serial.print(F("time "));
    Serial.print(millis());

    const auto& reply = [&] {
      mot1.SetQuery();
      return mot1.last_result().values;
    }();

    Serial.print(F("  mode "));
    Serial.print(static_cast<int>(reply.mode));
    Serial.print(F("  pos "));
    Serial.print(reply.position);
    Serial.print(F("  vel "));
    Serial.print(reply.velocity);
    Serial.print(F("  fault "));
    Serial.print(reply.fault);
    Serial.println();
  }

 private:
  Metro metro_{250};
} rpl_sndr_serial_print;

class NeoKeyCommandReceiver {
 public:
  NeoKeyCommandReceiver(uint16_t interval_ms) : neokey_{0x30, &Wire} {
    while (!neokey_.begin()) {
      Serial.println("Could not start NeoKey, check wiring?");
      delay(1000);
    }
    Serial.println(F("NeoKey started"));
  }

  void rcv() {
    if (!metro_.check()) return;

    const auto reading = neokey_.read();
    for (uint8_t i = 0; i < 4; i++) {
      if (reading & (1 << i)) {
        Serial.print("NeoKey #");
        Serial.print(i);
        Serial.println(" pressed");
        mot1.SetPosition({.position = 0.25 * i,
                          .maximum_torque = 16.0,
                          .velocity_limit = 16.0,
                          .accel_limit = 4.0},
                         &pm_fmt);
      }
    }
  }

 private:
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
  Serial.println(F("I2C0 started"));

  SPI.begin();
  Serial.println(F("SPI started"));

  while (1) {
    auto err_code = canfd_driver.begin(
        [] {
          ACAN2517FDSettings settings{ACAN2517FDSettings::OSC_40MHz,
                                      1000ll * 1000ll, DataBitRateFactor::x1};
          settings.mArbitrationSJW = 2;
          settings.mDriverTransmitFIFOSize = 1;
          settings.mDriverReceiveFIFOSize = 2;
          return settings;
        }(),
        [] { canfd_driver.isr(); });
    if (!err_code) {
      Serial.println("CAN FD driver started");
      break;
    }
    Serial.print(F("CAN FD driver begin failed, error code 0x"));
    Serial.println(err_code, HEX);
    delay(1000);
  }

  mot1.SetStop();
  Serial.println(F("moteus1 stopped"));
}

void loop() {
  delay(4);

  cmd_rcvr_neokey.rcv();
  rpl_sndr_serial_print.snd();
}
