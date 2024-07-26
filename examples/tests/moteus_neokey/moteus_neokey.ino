// Drive moteus of ID 1 and 2 with a NeoKey1x4 connected to the I2C0 port.
// Button #i commands moteuses to position 0.25 * i * (id % 2 ? 1 : -1).

#include <ACAN2517FD.h>
#include <Adafruit_NeoKey_1x4.h>
#include <Metro.h>
#include <Moteus.h>
#include <Wire.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>

PmFmt pm_fmt{.maximum_torque = Res::kFloat,
             .velocity_limit = Res::kFloat,
             .accel_limit = Res::kFloat};

PmCmd pm_cmd_template{
    .maximum_torque = 16.0, .velocity_limit = 16.0, .accel_limit = 4.0};

QFmt q_fmt{[] {
  QFmt fmt;
  fmt.abs_position = Res::kFloat;
  fmt.motor_temperature = Res::kInt16;
  fmt.trajectory_complete = Res::kInt8;
  return fmt;
}()};

Servo servos[] = {{1, &pm_fmt, &q_fmt}, {2, &pm_fmt, &q_fmt}};

template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (uint8_t i = 0; i < sizeof(servos) / sizeof(servos[0]); i++) {
    c(&servos[i]);
  }
}

class SerialPrintReplySender {
 public:
  void snd() {
    if (!metro_.check()) return;
    CommandAll([](Servo* servo) { servo->Print(); });
  }

 private:
  Metro metro_{250};
} serial_print_rpl_sndr;

class NeokeyCommandReceiver {
 public:
  void rcv() {
    if (!metro_.check()) return;

    const auto reading = neokey0.read();
    for (uint8_t i = 0; i < 4; i++) {
      if (reading & (1 << i)) {
        Serial.print("NeoKey #");
        Serial.print(i);
        Serial.println(" pressed");
        CommandAll([&](Servo* servo) {
          PmCmd pm_cmd{pm_cmd_template};
          pm_cmd.position = 0.25 * i * (servo->id_ % 2 == 0 ? 1.0 : -1.0);
          servo->SetPosition(pm_cmd);
        });
        return;
      }
    }
  }

 private:
  Metro metro_{50};
} neokey_cmd_rcvr;

void setup() {
  SerialInitializer.init();
  I2C0Initializer.init();
  SpiInitializer.init();
  Neokey0Initializer.init();
  CanFdInitializer.init();
  CommandAll([](Servo* servo) { servo->SetStop(); });
}

void loop() {
  neokey_cmd_rcvr.rcv();
  serial_print_rpl_sndr.snd();
}
