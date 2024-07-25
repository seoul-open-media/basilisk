// Drive moteus of ID 1 in sine wave motion.
// Code modified from source:
// https://github.com/mjbots/moteus-arduino/blob/main/examples/BasicControl/BasicControl.ino

#include <ACAN2517FD.h>
#include <Moteus.h>

// The following pins are selected for the seoul-open-media T4_CanFD board.
#define MCP2517_CS 10
#define MCP2517_INT 41

// The CAN FD driver (MCP2518FD) object using the ACAN2517FD Arduino library.
ACAN2517FD can(MCP2517_CS, SPI, MCP2517_INT);

// The moteus of ID 1.
Moteus moteus1(can, []() {
  Moteus::Options options;
  options.id = 1;
  return options;
}());

// The initial position of moteus1.
double moteus1_init_pos;

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

  // Clear all faults by sending a Stop command.
  moteus1.SetStop();
  Serial.println(F("moteus1 stopped"));

  // Save the initial position.
  moteus1.SetQuery();
  moteus1_init_pos = moteus1.last_result().values.position;
}

// Timers for Command and Reply to communicate with the moteus
// in regular time interval.
static uint32_t cmd_timer = 0;
static uint32_t rpl_timer = 0;

void loop() {
  // Send Command every 10ms.
  if (millis() > cmd_timer) {
    cmd_timer += 10;

    moteus1.SetPosition(Moteus::PositionMode::Command{
        .position = moteus1_init_pos + 0.5 * ::sin(millis() / 500.0)});
  }

  if (millis() > rpl_timer) {
    // Print Reply every 1s.
    rpl_timer += 1000;

    auto reply = moteus1.last_result().values;
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
