#pragma once

#include <Arduino.h>

#include <map>

#define TEENSY_UUID_REGISTER_0 (0x401F4410)
#define TEENSY_UUID_REGISTER_1 (0x401F4420)

uint64_t GetTeensyId() {
  union {
    uint64_t matome;
    uint32_t chunk[2];
  } teensyid;

  teensyid.chunk[0] = *(volatile uint32_t*)TEENSY_UUID_REGISTER_0;
  teensyid.chunk[1] = *(volatile uint32_t*)TEENSY_UUID_REGISTER_1;

  Serial.printf("TeensyID is 0x%08X", teensyid.chunk[1]);
  Serial.printf("%08X\n", teensyid.chunk[0]);

  return teensyid.matome;
}

inline static const std::map<uint64_t, uint8_t> teensyid_to_suid = {
    {0x331841D265F82946, 1},  //
    {0x3C18A1D265F8296A, 2},  //
    {0x134231D2677F0531, 3},  //
    {0x162111D764FE06E6, 4},  //
    {0x271311D764FE06E6, 5},  //
    {0x2D4731D2677F0531, 6},  //
    {0x212511D764FE06E6, 7},  //
    // {0x0000004CD2677F05, 8},   //
    {0x353B81D2677F054C, 9},   //
    {0x2A4231D2677F0531, 10},  //
    {0x313511D764FE06E6, 11},  //
    {0x422511D764FE06E6, 12},  //
    {0x1B2511D764FE06E6, 13},
};
