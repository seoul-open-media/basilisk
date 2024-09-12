#include <beat.h>
#include <initializers.h>
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

Servo servos[] = {
    {1, CANFD_BUS, &pm_fmt, &pm_cmd_template, PmCmdPosRelTo::Absolute, &q_fmt},
    {2, CANFD_BUS, &pm_fmt, &pm_cmd_template, PmCmdPosRelTo::Absolute, &q_fmt}};

template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (Servo& s : servos) {
    c(s);
  }
}

struct Command {
  enum class Mode : uint8_t { Stop, SetPosition } mode;

  struct Stop {
    bool init;
  } stop;

  struct SetPosition {
    double position;
    enum class FSMState : uint8_t { Init, Moving, Complete } fsm_state;
  } set_position;
} cmd;

void QueryExecuter() {
  CommandAll([](Servo& s) { s.Query(); });
}

void ExecuteStop() {
  if (cmd.stop.init) {
    Serial.println(F("ExecuteStop processing Init state"));
    CommandAll([](Servo& s) { s.Stop(); });
    cmd.stop.init = false;
  }
}

void ExecuteSetPosition() {
  using FSM = Command::SetPosition::FSMState;
  switch (cmd.set_position.fsm_state) {
    case FSM::Init: {
      Serial.println(F("ExecuteSetPosition processing Init state"));
      CommandAll([](Servo& s) { s.Position(cmd.set_position.position); });
      cmd.set_position.fsm_state = FSM::Moving;
    } break;
    case FSM::Moving: {
      Serial.println(F("ExecuteSetPosition processing Moving state"));
      for (Servo& s : servos) {
        if (s.trjcpt_ < 4) {
          return;
        }
      }
      CommandAll([](Servo& s) { s.Stop(); });
      cmd.set_position.fsm_state = FSM::Complete;
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

auto& neokey = specific::neokey3x4_i2c1;
void neokey_cb(uint16_t key) {
  Serial.print(F("Key rose: "));
  Serial.println(key);

  using C = Command;
  using M = C::Mode;
  auto& mode = cmd.mode;

  if (key == 0) {
    mode = M::Stop;
    cmd.stop.init = true;
  } else {
    mode = M::SetPosition;
    cmd.set_position.fsm_state = C::SetPosition::FSMState::Init;
    cmd.set_position.position = key * 0.25;
  }
}
void NeokeyCommandReceiver() { neokey.Run(); }

void SerialPrintReplySender() {
  CommandAll([](Servo& s) { s.Print(); });
}

Beat query_beat{10};
Beat command_beat{10};
Beat receive_beat{10};
Beat send_beat{500};

void setup() {
  SerialInitializer.init();
  I2C1Initializer.init();
  SpiInitializer.init();
  NeokeyInitializer.init(neokey);
  neokey.SetCommonRiseCallback(neokey_cb);
  CanFdInitializer.init(CANFD_BUS);

  CommandAll([](Servo& s) { s.Stop(); });
}

void loop() {
  if (receive_beat.Hit()) NeokeyCommandReceiver();
  if (query_beat.Hit()) QueryExecuter();
  if (command_beat.Hit()) CommandExecuter();
  if (send_beat.Hit()) SerialPrintReplySender();
}
