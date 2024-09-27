#pragma once

#include "../servo_units/basilisk.h"

#define XBEE_SERIAL Serial4
#define XBEE_PACKET_LEN 10  // NOT counting the 4 starting bytes.

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

    if (!receiving_) {
      if (XBEE_SERIAL.available() > 0) {
        uint8_t rbyte = XBEE_SERIAL.read();
        if (rbyte == 255) {
          start++;
        } else {
          start = 0;
        }

        if (start >= 4) receiving_ = true;
      }
    } else {
      if (XBEE_SERIAL.available() > 0 &&
          static_cast<uint32_t>(XBEE_SERIAL.available()) >= XBEE_PACKET_LEN) {
        RecvBuf temp_rbuf;
        for (uint8_t i = 0; i < XBEE_PACKET_LEN; i++) {
          temp_rbuf.raw_bytes[i] = XBEE_SERIAL.read();
        }

        // Filter out Command for different SUID immediately at reception time
        // to avoid timing mismatch with Executer Beat.
        if (temp_rbuf.decoded.suid != b_->cfg_.suid &&
            temp_rbuf.decoded.suid != 0) {
          waiting_parse_ = false;
        } else {
          memcpy(xbee_cmd_.raw_bytes, temp_rbuf.raw_bytes, XBEE_PACKET_LEN);
          waiting_parse_ = true;
        }
        receiving_ = false;
        start = 0;
      }
    }
  }

  static void Parse() {
    using C = Basilisk::Command;
    using M = C::Mode;
    static auto& x = xbee_cmd_.decoded;
    static auto& c = b_->cmd_;
    static auto& m = c.mode;

    c.oneshots = x.oneshots;

    const auto maybe_new_mode = static_cast<M>(x.mode);
    if (maybe_new_mode == M::DoPreset && x.u.preset.idx == 0) return;

    m = maybe_new_mode;
    switch (m) {
      case M::DoPreset: {
        c.do_preset.idx = x.u.preset.idx;
      } break;
      default:
        break;
    }
  }

  inline static union RecvBuf {
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
    uint8_t raw_bytes[XBEE_PACKET_LEN];
  } xbee_cmd_;

  inline static bool receiving_ = false;
  inline static bool waiting_parse_ = false;

 private:
  inline static Basilisk* b_ = nullptr;
};
