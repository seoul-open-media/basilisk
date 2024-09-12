#pragma once

#include "../helpers/imports.h"

// The following pins are selected for the SeoulOpenMedia T4_CanFD board v1.5.
// Only Busses 1 and 2 work for now.
#define MCP2518FD_CS_BUS1 10
#define MCP2518FD_INT_BUS1 41
#define MCP2518FD_CS_BUS2 9
#define MCP2518FD_INT_BUS2 42
#define MCP2518FD_CS_BUS3 0
#define MCP2518FD_INT_BUS3 43
#define MCP2518FD_CS_BUS4 2
#define MCP2518FD_INT_BUS4 44

// The CAN FD driver (MCP2518FD) objects using the ACAN2517FD Arduino library.
ACAN2517FD canfd_drivers[4] = {{MCP2518FD_CS_BUS1, SPI, MCP2518FD_INT_BUS1},
                               {MCP2518FD_CS_BUS2, SPI, MCP2518FD_INT_BUS2},
                               {MCP2518FD_CS_BUS3, SPI1, MCP2518FD_INT_BUS3},
                               {MCP2518FD_CS_BUS4, SPI1, MCP2518FD_INT_BUS4}};

void (*canfd_isrs[4])() = {
    [] { canfd_drivers[0].isr(); }, [] { canfd_drivers[1].isr(); },
    [] { canfd_drivers[2].isr(); }, [] { canfd_drivers[3].isr(); }};

class CanFdDriverInitializer {
 public:
  static bool Setup(const uint8_t& bus) {
    if (bus < 1 || bus > 4) {
      Serial.print("CanFdDriverInitializer: Unknown bus: ");
      Serial.println(bus);
      return false;
    } else if (bus <= 2) {
      SPI.begin();
    } else {
      SPI1.begin();
    }

    const auto err_code = canfd_drivers[bus - 1].begin(
        [] {
          ACAN2517FDSettings settings{ACAN2517FDSettings::OSC_40MHz,
                                      1000ll * 1000ll, DataBitRateFactor::x1};
          settings.mArbitrationSJW = 2;
          settings.mDriverTransmitFIFOSize = 1;
          settings.mDriverReceiveFIFOSize = 2;

          return settings;
        }(),
        canfd_isrs[bus - 1]);

    if (err_code) {
      Serial.print("CanFdDriverInitializer: CAN FD driver on bus ");
      Serial.print(bus);
      Serial.print(" begin failed, error code 0x");
      Serial.println(err_code, HEX);
      return false;
    }

    Serial.print("CanFdDriverInitializer: CAN FD driver on bus ");
    Serial.print(bus);
    Serial.println(F(" started"));
    return true;
  }
};
