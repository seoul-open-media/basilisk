#pragma once

#include "../servo_units/basilisk.h"

#define XBEE_SERIAL Serial4
#define XBEE_PACKET_LEN 46  // NOT counting the 4 starting bytes.

class XbeeCommandReceiver {
  using C = Basilisk::Command;
  using M = C::Mode;

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
    static RecvBuf temp_rbuf;
    static uint8_t buf_idx;
    static uint32_t start_time_us;

    if (!receiving_) {
      if (XBEE_SERIAL.available() > 0) {
        uint8_t rbyte = XBEE_SERIAL.read();
        if (rbyte == 255) {
          start++;
        } else {
          start = 0;
        }

        if (start < 4) return;

        receiving_ = true;
        buf_idx = 0;
        start_time_us = micros();
      }
    }

    if (micros() > start_time_us + 2000) {
      receiving_ = false;
      start = 0;
      return;
    }

    while (XBEE_SERIAL.available() > 0) {
      if (buf_idx < XBEE_PACKET_LEN) {
        temp_rbuf.raw_bytes[buf_idx] = XBEE_SERIAL.read();
        buf_idx++;
      } else {
        break;
      }
    }

    if (buf_idx < XBEE_PACKET_LEN) return;

    /* Print for debug */ {
      Serial.print("AheID ");
      Serial.print(b_->cfg_.suid);
      Serial.print(" received full XbeeCommand within 2ms since start bytes");
      Serial.println();

      Serial.print("Command bytes -> ");
      for (size_t i = 0; i < XBEE_PACKET_LEN; i++) {
        Serial.print(temp_rbuf.raw_bytes[i]);
        Serial.print(", ");
      }
      Serial.println();

      Serial.print("Mode -> ");
      Serial.print(temp_rbuf.decoded.mode);
      Serial.println();

      if (temp_rbuf.decoded.mode == static_cast<uint8_t>(M::DoPreset)) {
        Serial.print("Preset indices for all Ahes -> ");
        for (uint8_t i = 0; i < 13; i++) {
          Serial.print(temp_rbuf.decoded.u.do_preset.idx[i]);
          Serial.print(", ");
        }
        Serial.println();

        Serial.print("My Preset index -> ");
        Serial.print(temp_rbuf.decoded.u.do_preset.idx[b_->cfg_.suid]);
        Serial.println();
      }

      Serial.print("Is this Command for me? ");
      Serial.print((temp_rbuf.decoded.suid & (1 << (b_->cfg_.suid - 1)))
                       ? "True"
                       : "False");
    }

    if (!(temp_rbuf.decoded.suid & (1 << (b_->cfg_.suid - 1)))) {
      waiting_parse_ = false;
    } else {
      memcpy(xbee_cmd_.raw_bytes, temp_rbuf.raw_bytes, XBEE_PACKET_LEN);
      waiting_parse_ = true;
    }

    receiving_ = false;
    start = 0;
  }

  static void Parse() {
    static auto& x = xbee_cmd_.decoded;
    static auto& c = b_->cmd_;
    static auto& m = c.mode;

    if (x.oneshots) {
      c.oneshots = x.oneshots;

      if (x.oneshots & (1 << 1)) {
        c.set_base_yaw.offset = static_cast<double>(x.u.set_base_yaw.offset);
      }

      return;  // Oneshot- and Mode- Commands are NOT processed from
               // a single Command since there are Oneshots that
               // require additional values.
    }

    const auto maybe_new_mode = static_cast<M>(x.mode);
    if (maybe_new_mode == M::DoPreset && x.u.do_preset.idx == 0)
      return;  // Do not even switch Mode.  Previous DoPreset Command
               // execution's future-chaining can be happening now.

    m = maybe_new_mode;
    switch (m) {
      case M::DoPreset: {
        c.do_preset.idx = x.u.do_preset.idx[b_->cfg_.suid - 1];

        Serial.print("Preset index ");
        Serial.print(c.do_preset.idx);
        Serial.println();
      } break;
      case M::Pivot_Init: {
        c.pivot.bend[IDX_L] = static_cast<double>(x.u.pivot.bend_l);
        c.pivot.bend[IDX_R] = static_cast<double>(x.u.pivot.bend_r);
        c.pivot.didimbal = static_cast<bool>(x.u.pivot.didimbal);
        c.pivot.speed = static_cast<double>(x.u.pivot.speed);
        c.pivot.stride = static_cast<double>(x.u.pivot.stride);
        c.pivot.tgt_yaw = [](Basilisk*) {
          return static_cast<double>(x.u.pivot.tgt_yaw);
        };
        c.pivot.min_dur = 0;
        c.pivot.max_dur = 6000;
        c.pivot.exit_condition = nullptr;
        c.pivot.acclim = 1.0;
        c.pivot.exit_to_mode = M::Idle_Init;
      } break;
      default: {
        Serial.print("XbeeCommandReceiver: Mode ");
        Serial.print(x.mode);
        Serial.print(" is NOT registered");
        Serial.println();
      } break;
    }
  }

  inline static union RecvBuf {
    struct Decoded {
      uint16_t suid;
      uint8_t oneshots;
      uint8_t mode;
      union {
        struct {
          float offset;
        } __attribute__((packed)) set_base_yaw;
        struct {
          uint16_t idx[13];  // The Goguma version of DoPreset protocol.
        } __attribute__((packed)) do_preset;
        struct {
          float bend_l;
          float bend_r;
          uint8_t didimbal;
          float speed;
          float stride;
          float tgt_yaw;
        } __attribute__((packed)) pivot;
      } u;
    } __attribute__((packed)) decoded;
    uint8_t raw_bytes[XBEE_PACKET_LEN];
  } xbee_cmd_;

  inline static bool receiving_ = false;
  inline static bool waiting_parse_ = false;

 private:
  inline static Basilisk* b_ = nullptr;
};
