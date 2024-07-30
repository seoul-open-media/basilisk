// Expected output:
// Time(s)  : 0       1       2       3       4 ...
//            |       |       |       |       | ...
// Run      : c       |       |       |       | ...
// Thread   : v v v v v v v v |       |       | ...
// Main loop: v   v   v   v   v   v   v   v   | ...
// c is chosen random every 4s.
// If c == 1, v increments every print from Thread.
// If c == 2, v decrements every print from Thread.
// If c == 0, v resets to zero.

#include <Metro.h>
#include <TeensyThreads.h>

Threads::Mutex serial_mutex;

class ServoUnit {
 public:
  struct {
    volatile int value;
  } cmd_;

  Threads::Mutex cmd_mutex_;
} su;

class CommandReceiver {
 public:
  CommandReceiver(ServoUnit& su) : su_{su} {}

  void Run(const uint32_t& interval = 4000) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        const auto received = random(3);
        {
          Threads::Scope serial_lock{serial_mutex};
          Serial.print(F("Run: "));
          Serial.println(received);
        }
        switch (received) {
          case 0: {
            {
              Threads::Scope cmd_lock{su_.cmd_mutex_};
              su_.cmd_.value = 0;
            }
            for (size_t i = 0; i < 8; i++) {
              {
                Threads::Scope serial_lock{serial_mutex};
                Serial.print(F("Thread: "));
                Threads::Scope cmd_lock{su_.cmd_mutex_};
                Serial.println(su_.cmd_.value);
              }
              delay(250);
            }
          } break;
          case 1: {
            for (size_t i = 0; i < 8; i++) {
              {
                Threads::Scope cmd_lock{su_.cmd_mutex_};
                su_.cmd_.value++;
                Threads::Scope serial_lock{serial_mutex};
                Serial.print(F("Thread: "));
                Serial.println(su_.cmd_.value);
              }
              delay(250);
            }
          } break;
          case 2: {
            for (size_t i = 0; i < 8; i++) {
              {
                Threads::Scope cmd_lock{su_.cmd_mutex_};
                su_.cmd_.value--;
                Threads::Scope serial_lock{serial_mutex};
                Serial.print(F("Thread: "));
                Serial.println(su_.cmd_.value);
              }
              delay(250);
            }
          } break;
          default:
            break;
        }
      }
    }
  }

  ServoUnit& su_;
} cr{su};

class ReplySender {
 public:
  ReplySender(ServoUnit& su) : su_{su} {}

  void Run(const uint16_t interval = 500) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        Threads::Scope serial_lock{serial_mutex};
        Serial.print("Main loop: ");
        Threads::Scope cmd_lock{su.cmd_mutex_};
        Serial.println(su.cmd_.value);
      }
    }
  }

  ServoUnit& su_;
} rs{su};

void setup() {
  Serial.begin(115200);
  randomSeed(0xDEADFACE);
  threads.addThread([] { cr.Run(); });
  threads.addThread([] { rs.Run(); });
}

void loop() { yield(); }
