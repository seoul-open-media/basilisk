// Drive moteus of ID 1 and 2 on Bus 1 in sine wave motion. Wiggle wiggle.

#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <TeensyThreads.h>
#include <initializers.h>
#include <servo.h>

#define CANFD_BUS 1  // Only Buses 1 and 2 work for T4_CanFd board v.1.5.

class SineServoUnit {
 public:
  SineServoUnit() : servos_{{1, CANFD_BUS}, {2, CANFD_BUS}} {}

  template <typename ServoCommand>
  void CommandUnit(ServoCommand c) {
    for (Servo& s : servos_) {
      c(s);
    }
  }

  // Query and Command in a unified Executer to avoid unnecessary
  // syncronization problem since generally Replies are needed for Commands,
  // and they should be executed at same frequency anyway.
  // Executer is placed inside ServoUnit for Teensy version since
  // bulk execution is not implemented in the Moteus Arduino library.
  void Executer(const uint32_t& interval = 10) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        CommandUnit([](Servo& s) { s.Query(); });
        CommandUnit([](Servo& s) { s.Position(0.25 * sin(millis() / 250.0)); });
      }
    }
  }

  // Place ReplySender inside ServoUnit as a method rather making a class for it
  // for this simple kind of scenario.
  void SerialPrintReplySender(const uint32_t& interval = 100) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        CommandUnit([](Servo& servo) { servo.Print(); });
      }
    }
  }

  Servo servos_[2];
} sine_su;

void setup() {
  SerialInitializer.init();

  // Begin [Teensy]-[CAN FD drivers] connection.
  if (CANFD_BUS == 1 || CANFD_BUS == 2) {
    SpiInitializer.init();
  } else if (CANFD_BUS == 3 || CANFD_BUS == 4) {
    Serial.println(F("Only Buses 1 and 2 work for T4_CanFd board v.1.5"));
    Spi1Initializer.init();
  } else {
    while (1) {
      Serial.println("Invalid CAN FD Bus");
      delay(1000);
    }
  }

  // Begin the CAN FD driver.
  CanFdInitializer.init(CANFD_BUS);

  // Clear all faults by sending Stop commands, and save the initial positions.
  // sine_su.CommandUnit([](Servo* servo) { servo->Stop(); });
  sine_su.CommandUnit([](Servo& s) { s.Stop(); });

  threads.addThread([] { sine_su.Executer(); });
  threads.addThread([] { sine_su.SerialPrintReplySender(); });
}

void loop() { yield(); }
