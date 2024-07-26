// Drive moteus of ID 1 in sine wave motion.
// Code modified from source:
// https://github.com/mjbots/moteus-arduino/blob/main/examples/BasicControl/BasicControl.ino

#include <ACAN2517FD.h>
#include <Moteus.h>

// The following pins are selected for the seoul-open-media T4_CanFD board.
#define MCP2518FD_CS 10
#define MCP2518FD_INT 41

// The CAN FD driver (MCP2518FD) object using the ACAN2517FD Arduino library.
ACAN2517FD canfd_driver(MCP2518FD_CS, SPI, MCP2518FD_INT);

// The moteus of ID 1.
Moteus mot1(canfd_driver, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

// The initial position of moteus1.
double mot1_init_pos;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    Serial.println(F("Serial failed"));
    delay(1000);
  }
  Serial.println(F("Serial started"));

  // The Teensy and the CAN FD drivers are connected via SPI.
  SPI.begin();
  Serial.println(F("SPI started"));

  // Setup the CAN FD driver.
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

  // Clear all faults by sending a Stop command.
  mot1.SetStop();
  Serial.println(F("moteus1 stopped"));

  // Save the initial position.
  mot1_init_pos = [&] {
    mot1.SetQuery();
    return mot1.last_result().values.position;
  }();
}

// Timers for Command and Reply to communicate with the moteus
// in regular time interval.
static uint32_t cmd_timer = 0;
static uint32_t rpl_timer = 0;

void loop() {
  // Send Command every 10ms.
  if (millis() > cmd_timer) {
    cmd_timer += 10;

    mot1.SetPosition(
        {.position = mot1_init_pos + 0.5 * ::sin(millis() / 500.0)});
  }

  // Print Reply every 1s.
  if (millis() > rpl_timer) {
    rpl_timer += 1000;

    auto reply = mot1.last_result().values;
    Serial.print(F("time "));
    Serial.print(millis());
    Serial.print(F("  mode "));
    Serial.print(static_cast<int>(reply.mode));
    Serial.print(F("  pos "));
    Serial.print(reply.position);
    Serial.print(F("  vel "));
    Serial.print(reply.velocity);
    Serial.println();
  }
}
