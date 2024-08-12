#include <stddef.h>
#include <stdint.h>

namespace summary {

// CAN FD frame representation used in the ACAN2517FD Arduino library.
class CANFDMessage {};

// The CAN FD driver class from the ACAN2517FD Arduino library,
// compatible with MCP2518FD.
class ACAN2517FD {
 public:
  bool tryToSend(const CANFDMessage&) {
    /* Send CAN FD message to moteus and return successfulness. */
  }

  bool receive(CANFDMessage&) {
    /* Receive CAN FD message to reference argument
     * and return successfulness. */
  }

  bool available() { /* return canfdReceiveBuffer.count() > 0 */ }

  void poll() {
    /* Begin SPI transaction.
     * Handle interrupts and do some SPI stuff.
     * End SPI transaction. */
  }
};

struct CanData {
  uint8_t data[64] = {};
  uint8_t size = 0;
};

// Helper class to append CAN FD messages to CanData.
class WriteCanData {
 public:
  WriteCanData(CanData* frame) : data_{frame->data}, size_{&frame->size} {}

  template <typename WriteAs, typename RawValue>
  void Write(RawValue value_in) {
    auto value = static_cast<WriteAs>(value_in);

    if (sizeof(value) + *size_ > 64) abort();

    memcpy(&data_[*size_], reinterpret_cast<const char*>(&value),
           sizeof(value));
    *size_ += sizeof(value);
  }

  /* Write-variant methods */

 private:
  uint8_t* const data_;
  uint8_t* const size_;
};

struct CanFdFrame {
  uint8_t data[64] = {};
  uint8_t size = 0;

  int8_t bus;
  int8_t destination;  // ID of moteus to which frame is sent.
  bool reply_required;
};

struct Query {
  struct Result {};
  struct Format {};
  static uint8_t Make(WriteCanData* frame, const Format& fmt) {
    uint8_t reply_size = 0;

    /* Make CAN FD frame based on fmt and write to *frame
     * and update reply_size. */

    return reply_size;
  }
};

struct EmptyMode {
  struct Command {};
  struct Format {};
  static uint8_t Make(WriteCanData*, const Command&, const Format&) {
    /* Do nothing. */

    return 0;
  }
};

struct PositionMode {
  struct Command {};
  struct Format {};
  static uint8_t Make(WriteCanData* frame, const Command& cmd,
                      const Format& fmt) {
    /* Make CAN FD frame based on cmd and fmt and write to *frame. */

    return 0;
  }
};

struct StopMode {
  struct Command {};
  struct Format {};
  static uint8_t Make(WriteCanData* frame, const Command&, const Format&) {
    /* Make CAN FD frame that sets Mode register to Mode::kStopped
     * and write to *frame */

    return 0;
  }
};

class Moteus {
 public:
  struct Options {
    int8_t id;
    Query::Format default_q_fmt;
    bool query_by_default = true;
  };

  Moteus(ACAN2517FD& canfd_driver, const Options& options)
      : canfd_driver_{canfd_driver}, options_{options} {
    /* Setup default Query frame (default_query_frame_data_ and
     * default_query_frame_size_) using options_.default_q_fmt. */
    CanData query_frame;
    WriteCanData query_frame_writer{&query_frame};
    Query::Make(&query_frame_writer, options_.default_q_fmt);
    default_query_frame_size_ = query_frame.size;
    default_query_frame_data_ = reinterpret_cast<char*>(
        realloc(default_query_frame_data_, default_query_frame_size_));
    memcpy(default_query_frame_data_, &query_frame.data[0],
           default_query_frame_size_);
  }

  struct Result {
    uint32_t timestamp = 0;
    CanFdFrame frame;
    Query::Result values;
  };

  const Result& last_result() const { return last_query_reply_; }

  // Query

  CanFdFrame MakeQuery(Query::Format* q_fmt_override = nullptr) {
    return MakeFrame(
        EmptyMode{}, {}, {},
        q_fmt_override == nullptr ? &options_.default_q_fmt : q_fmt_override);
  }

  bool SetQuery(Query::Format* q_fmt_override = nullptr) {
    return ExecuteSingleCommand(MakeQuery(q_fmt_override));
  }

  // PositionMode

  CanFdFrame MakePosition(const PositionMode::Command& pm_cmd,
                          const PositionMode::Format* pm_fmt_override = nullptr,
                          const Query::Format* q_fmt_override = nullptr) {
    return MakeFrame(
        PositionMode{}, pm_cmd,
        pm_fmt_override == nullptr ? PositionMode::Format{} : *pm_fmt_override,
        q_fmt_override);
  }

  bool SetPosition(const PositionMode::Command& pm_cmd,
                   const PositionMode::Format* pm_fmt_override = nullptr,
                   const Query::Format* q_fmt_override = nullptr) {
    return ExecuteSingleCommand(
        MakePosition(pm_cmd, pm_fmt_override, q_fmt_override));
  }

  // StopMode

  CanFdFrame MakeStop(const Query::Format* q_fmt_override = nullptr) {
    return MakeFrame(StopMode{}, {}, {}, q_fmt_override);
  }

  bool SetStop(Query::Format* q_fmt_override = nullptr) {
    return ExecuteSingleCommand(MakeStop(q_fmt_override));
  }

  // Execution methods

  bool Poll() {
    canfd_driver_.poll();
    if (!canfd_driver_.available()) return false;

    CANFDMessage rx_msg;
    canfd_driver_.receive(rx_msg);

    /* if (ID mismatch) return false
     * last_query_reply_.timestamp = now
     * last_query_reply_.frame = translate(rx_msg)
     * last_query_reply_.values = Query::Parse(last_query_reply_.frame) */

    return true;
  }

  bool BeginSingleCommand(const CanFdFrame& frame) {
    CANFDMessage can_msg;
    /* Configure can_msg based on frame. */

    // "To work even when the ACAN2517FD doesn't have functioning
    // interrupts, we will just poll it before and after attempting to
    // send our message.  This slows things down, but we're on an
    // Arduino, so who cares?"
    canfd_driver_.poll();
    canfd_driver_.tryToSend(can_msg);
    canfd_driver_.poll();

    return frame.reply_required;
  }

  bool ExecuteSingleCommand(const CanFdFrame& frame) {
    const bool reply_required = BeginSingleCommand(frame);

    if (!reply_required) {
      return false;
    }

    bool got_reply = false;

    while (true) {
      /* if (timeout(options_.min_rcv_wait_us)) return false; */

      // Return if got Reply, and have no more queued up.
      if (Poll()) {
        got_reply = true;
      } else if (got_reply) {
        return true;
      }
    }
  }

 private:
  enum ReplyMode { kNoReply, kReplyRequired };

  CanFdFrame DefaultFrame(ReplyMode reply_mode = kReplyRequired) {}

  template <typename CommandType>
  CanFdFrame MakeFrame(const CommandType&,
                       const typename CommandType::Command& cmd,
                       const typename CommandType::Format& fmt,
                       const Query::Format* q_fmt_override = nullptr) {
    auto frame = DefaultFrame(q_fmt_override != nullptr   ? kReplyRequired
                              : options_.query_by_default ? kReplyRequired
                                                          : kNoReply);

    WriteCanData frame_writer{frame.data, &frame.size};
    CommandType::Make(&frame_writer, cmd, fmt);

    if (q_fmt_override) {
      Query::Make(&frame_writer, *q_fmt_override);
    } else if (options_.query_by_default) {
      memcpy(&frame.data[frame.size], default_query_frame_data_,
             default_query_frame_size_);
      frame.size += default_query_frame_size_;
    }

    return frame;
  }

  ACAN2517FD& canfd_driver_;
  const Options options_;
  Result last_query_reply_;
  char* default_query_frame_data_ = nullptr;
  size_t default_query_frame_size_ = 0;
};

}  // namespace summary
