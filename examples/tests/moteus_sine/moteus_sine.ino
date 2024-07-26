// Drive moteus of ID 1 and 2 in sine wave motion.
// Wiggle wiggle.

#include <ACAN2517FD.h>
#include <Metro.h>
#include <Moteus.h>
#include <initializers.h>
#include <servo.h>

Servo servos[] = {{1}, {2}};

template <typename ServoCommand>
void CommandAll(ServoCommand c) {
  for (uint8_t i = 0; i < sizeof(servos) / sizeof(servos[0]); i++) {
    c(&servos[i]);
  }
}

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();    // [Teensy]-[CAN FD drivers] connection.
  CanFdInitializer.init();  // Setup the CAN FD driver.

  // Clear all faults by sending Stop commands, and save the initial positions.
  CommandAll([](Servo* servo) {
    servo->SetStop();
    delay(10);
    servo->SetBasePosition();
    Serial.print(F("Servo "));
    Serial.print(servo->id_);
    Serial.println(F(" stopped and base position set"));
  });
}

Metro cmd_metro{10};
Metro rpl_metro{250};

void loop() {
  // Send Command every 10ms.
  if (cmd_metro.check()) {
    servos[0].controller_.SetPosition(
        {.position = servos[0].base_pos_ + 0.25 * ::sin(millis() / 250.0)});
    servos[1].controller_.SetPosition(
        {.position = servos[1].base_pos_ + 0.5 * ::sin(millis() / 125.0)});
  }

  // Print Reply every 1s.
  if (rpl_metro.check()) {
    CommandAll([](Servo* servo) { servo->Print(); });
  }
}
