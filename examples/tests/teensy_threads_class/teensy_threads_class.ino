// Make sure registering threads to `threads` by `threads.addThread()` happens
// inside the `setup()` function. Do not register threads in class constructors.
// Recommended that all variables manipulated by threads be declared as
// volatile.

#include <Metro.h>
#include <TeensyThreads.h>

class ServoUnit {
 public:
  struct {
    volatile int value;
  } cmd_;

  Threads::Mutex mutex_;
} su;

class CommandReceiver {
 public:
  CommandReceiver(ServoUnit* su) : su_{su} {}

  void Run(const uint32_t& interval = 4000) {
    Metro metro{interval};
    while (1) {
      if (metro.check()) {
        auto received = random(3);
        Serial.print(F("Run: "));
        Serial.println(received);

        switch (received) {
          case 0: {
            su_->cmd_.value = 0;
          } break;
          case 1: {
            for (size_t i = 0; i < 8; i++) {
              {
                Threads::Scope lock{su.mutex_};
                su_->cmd_.value++;
                Serial.print(F("Thread: "));
                Serial.println(su_->cmd_.value);
              }
              delay(250);
            }
          } break;
          case 2: {
            for (size_t i = 0; i < 8; i++) {
              {
                Threads::Scope lock{su.mutex_};
                su_->cmd_.value--;
                Serial.print(F("Thread: "));
                Serial.println(su_->cmd_.value);
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

  ServoUnit* su_;
} cr{&su};

void setup() {
  Serial.begin(115200);
  randomSeed(0xDEADFACE);
  threads.addThread([](void* self) { cr.Run(); });
}

void loop() {
  {
    Serial.print("Main loop: ");
    Threads::Scope lock{su.mutex_};
    Serial.println(su.cmd_.value);
  }

  delay(500);
}
