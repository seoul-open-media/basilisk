// Drive moteus of ID 1 with a NeoKey1x4 connected to the I2C0 port.
// Button #i commands moteus to position 0.25 * i.

#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>

#define MCP2517_CS 10
#define MCP2517_INT 41
ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  options.query_format = [] {
    Moteus::Query::Format fmt;
    fmt.abs_position = mjbots::moteus::Resolution::kFloat;
    fmt.motor_temperature = mjbots::moteus::Resolution::kInt16;
    fmt.trajectory_complete = mjbots::moteus::Resolution::kInt8;
    return fmt;
  }();
  return options;
}());

Moteus::PositionMode::Format cmd_fmt{
    .velocity = mjbots::moteus::Resolution::kIgnore,
    .maximum_torque = mjbots::moteus::Resolution::kFloat,
    .velocity_limit = mjbots::moteus::Resolution::kFloat,
    .accel_limit = mjbots::moteus::Resolution::kFloat};

class SerialPrintReplySender {
 public:
  void snd() {
    if (!metro_.check()) return;

    Serial.print(F("time "));
    Serial.print(millis());

    const auto& reply = [&] {
      moteus1.SetQuery();
      return moteus1.last_result().values;
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
        moteus1.SetPosition(Moteus::PositionMode::Command{.position = 0.25 * i},
                            &cmd_fmt);
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

  ACAN2517FDSettings settings(ACAN2517FDSettings::OSC_40MHz, 1000ll * 1000ll,
                              DataBitRateFactor::x1);
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

  moteus1.SetStop();
  Serial.println(F("All moteus stopped"));
  moteus1.SetPositionWaitComplete(Moteus::PositionMode::Command{.position = 0},
                                  0.1, &cmd_fmt);
  Serial.println(F("moteus1 sent to position 0"));
}

void loop() {
  delay(10);

  cmd_rcvr_neokey.rcv();
  rpl_sndr_serial_print.snd();
}
