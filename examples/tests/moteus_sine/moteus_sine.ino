// Drive moteus of ID 1 and 2 in sine wave motion. Wiggle wiggle.

#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <TeensyThreads.h>
#include <initializers.h>
#include <servo.h>

class SineServoUnit {
 public:
  SineServoUnit() : servos_{{1}, {2}} {}

  template <typename ServoCommand>
  void CommandAll(ServoCommand c) {
    c(&servos_[0]);
    c(&servos_[1]);
  }

  // Place ReplySender inside ServoUnit as a method rather making a class for it
  // for this simple kind of scenario.
  void SerialPrintReplySender(const uint32_t& interval = 250) {  // Default 4Hz.
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        CommandAll([](Servo* servo) { servo->Print(); });
      }
    }
  }

  // Query and Command in a unified Executer to avoid unnecessary
  // syncronization problem since generally Replies are needed for Commands,
  // and they should be executed at same frequency anyway.
  // Executer is placed inside ServoUnit for Teensy version since
  // bulk execution is not implemented in the Moteus Arduino library.
  void Executer(const uint32_t& interval = 10) {  // Default 100Hz.
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        CommandAll([](Servo* servo) { servo->Query(); });

        servos_[0].Position({.position = 0.25 * ::sin(millis() / 250.0)});
        servos_[1].Position({.position = 0.5 * ::sin(millis() / 125.0)});
      }
    }
  }

  Servo servos_[2];
} sine_su;

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();    // [Teensy]-[CAN FD drivers] connection.
  CanFdInitializer.init();  // Setup the CAN FD driver.

  // Clear all faults by sending Stop commands, and save the initial positions.
  sine_su.CommandAll([](Servo* servo) { servo->Stop(); });

  threads.addThread([] { sine_su.Executer(); });
  threads.addThread([] { sine_su.SerialPrintReplySender(); });
}

void loop() {}
