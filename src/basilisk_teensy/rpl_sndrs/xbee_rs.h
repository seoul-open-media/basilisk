#pragma once

#include "../servo_units/basilisk.h"

#define XBEE_SERIAL Serial4
#define XBEE_PACKET_LEN 46  // NOT counting the 4 starting bytes.

class XbeeReplySender {
  void Send(uint8_t* src) {}

  inline static union RecvBuf {
    struct Decoded {
      uint8_t suid;
      uint8_t mode;
      union {
        struct {
          float offset;
        } __attribute__((packed)) set_base_yaw;
        struct {
          uint16_t idx;
        } __attribute__((packed)) preset;
      } u;
    } __attribute__((packed)) decoded;
    uint8_t raw_bytes[XBEE_PACKET_LEN];
  } xbee_cmd_;
};
