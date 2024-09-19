#pragma once

#include "../servo_units/basilisk.h"

#define XBEE_SERIAL Serial4

class XbeeCommandReceiver {
 public:
  bool Setup(Basilisk* b) {
    if (!b) {
      Serial.println("XbeeCommandReceiver: Null pointer to Basilisk");
      return false;
    }
    b_ = b;
    Serial.println("XbeeCommandReceiver: Registered reference to Basilisk");

    XBEE_SERIAL.begin(115200);
    if (!XBEE_SERIAL) {
      Serial.println("XbeeCommandReceiver: XBEE_SERIAL(Serial4) begin failed");
      return false;
    }
    Serial.println("XbeeCommandReceiver: Setup complete");
    return true;
  }

  void Run() {
    static uint8_t start = 0;

    if (!receiving) {
      if (XBEE_SERIAL.available()) {
        uint8_t rbyte = XBEE_SERIAL.read();
        if (rbyte == 255) {
          start++;
        } else {
          start = 0;
        }

        if (start < 4) return;
        receiving = true;
      }
    } else {
      if (XBEE_SERIAL.available() >= sizeof(xbee_cmd_)) {
        for (int i = 0; i < sizeof(xbee_cmd_); i++) {
          xbee_cmd_.raw_bytes[i] = XBEE_SERIAL.read();
        }
        receiving = false;
        start = 0;
      }
    }
  }

  static void Parse() {
    if (receiving) {
      waiting_parse = true;
      return;
    }

    using C = Basilisk::Command;
    using M = C::Mode;
    static auto& c = b_->cmd_;
    static auto& m = c.mode;

    if (xbee_cmd_.decoded.oneshots & 1) {
    }
  }

  static union RecvBuf {
    struct Decoded {
      uint8_t suid;
      uint8_t oneshots;
      uint8_t mode;
      union {
        struct {
          uint16_t idx;
        } __attribute__((packed)) preset;
      } u;
    } __attribute__((packed)) decoded;
    uint8_t raw_bytes[sizeof(decoded)];
  } xbee_cmd_;

  inline static bool receiving = false;
  inline static bool waiting_parse = false;

 private:
  inline static Basilisk* b_ = nullptr;
};
