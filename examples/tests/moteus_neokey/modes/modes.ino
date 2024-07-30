#include <Metro.h>
#include <initializers.h>
#include <neokey.h>
#include <servo.h>
#include <specific/neokey3x4_i2c1.h>

#define CANFD_BUS 1

PmFmt pm_fmt{.maximum_torque = Res::kFloat,
             .velocity_limit = Res::kFloat,
             .accel_limit = Res::kFloat};

PmCmd pm_cmd_template{
    .maximum_torque = 32.0, .velocity_limit = 32.0, .accel_limit = 16.0};

QFmt q_fmt{[] {
  QFmt fmt;
  fmt.abs_position = Res::kFloat;
  fmt.motor_temperature = Res::kInt16;
  fmt.trajectory_complete = Res::kInt8;
  return fmt;
}()};

Servo servos[] = {{1, CANFD_BUS, &pm_fmt, &pm_cmd_template,
                   CommandPositionRelativeTo::Absolute, &q_fmt},
                  {2, CANFD_BUS, &pm_fmt, &pm_cmd_template,
                   CommandPositionRelativeTo::Absolute, &q_fmt}};

template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (Servo& s : servos) {
    c(s);
  }
}

struct Command {
  enum class Mode : uint8_t { Stop, DZero, SetPosition, Sine } mode;

  struct Stop {
    bool init;
  } stop;

  struct SetPosition {
    double position;
    enum class Progress : uint8_t { init, moving, complete, stopped } progress;
  } set_position;
} cmd;

void QueryExecuter() {
  CommandAll([](Servo& s) { s.Query(); });
}

void ExecuteStop() {
  if (cmd.stop.init) {
    Serial.println(F("ExecuteStop processing init state"));
    CommandAll([](Servo& s) { s.Stop(); });
    cmd.stop.init = false;
  }
}

void ExecuteSetPosition() {
  using P = Command::SetPosition::Progress;
  switch (cmd.set_position.progress) {
    case P::init: {
      Serial.println(F("ExecuteSetPosition processing init state"));
      CommandAll([](Servo& s) { s.Position(cmd.set_position.position); });
      cmd.set_position.progress = P::moving;
    } break;
    case P::moving: {
      Serial.println(F("ExecuteSetPosition processing moving state"));
      for (Servo& s : servos) {
        if (s.trjcpt_ < 4) {
          return;
        }
      }
      cmd.set_position.progress = P::complete;
    } break;
    case P::complete: {
      Serial.println(F("ExecuteSetPosition processing complete state"));
      CommandAll([](Servo& s) { s.Stop(); });
      cmd.set_position.progress = P::stopped;
    } break;
    default:
      break;
  }
}

void CommandExecuter() {
  using M = Command::Mode;
  if (cmd.mode == M::Stop) {
    ExecuteStop();
  } else if (cmd.mode == M::SetPosition) {
    ExecuteSetPosition();
  }
}

NeoKey1x4Callback neokey_cb(keyEvent evt) {
  if (evt.bit.EDGE == SEESAW_KEYPAD_EDGE_RISING) {
    auto key = evt.bit.NUM;

    Serial.print(F("Rise: "));
    Serial.println(key);

    using C = Command;
    using M = C::Mode;
    auto& mode = cmd.mode;

    if (key < 4) {
      mode = M::Stop;
      cmd.stop.init = true;
    } else {
      mode = M::SetPosition;
      cmd.set_position.progress = C::SetPosition::Progress::init;
      cmd.set_position.position = key * 0.25;
    }
  }

  return 0;
}

auto& neokey = specific::neokey3x4_i2c1;
void NeokeyCommandReceiver() { neokey.read(); }

void SerialPrintReplySender() {
  Serial.print("cmd.mode = ");
  Serial.println(cmd.mode == Command::Mode::Stop          ? "Stop"
                 : cmd.mode == Command::Mode::SetPosition ? "SetPosition"
                                                          : "Else");
  CommandAll([](Servo& s) { s.Print(); });
}

Metro query_metro{10};
Metro command_metro{10};
Metro receive_metro{10};
Metro send_metro{250};

void setup() {
  SerialInitializer.init();
  I2C0Initializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);
  NeokeyInitializer.init(neokey);
  neokey.registerCallbackAll(neokey_cb);

  CommandAll([](Servo& s) { s.Stop(); });
}

void loop() {
  if (receive_metro.check()) NeokeyCommandReceiver();
  if (query_metro.check()) QueryExecuter();
  if (command_metro.check()) CommandExecuter();
  if (send_metro.check()) SerialPrintReplySender();
}
