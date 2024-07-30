#include <Metro.h>
#include <TeensyThreads.h>
#include <initializers.h>
#include <servo.h>

#define CANFD_BUS 1

Servo servo{1, CANFD_BUS};

Threads::Mutex serial_mutex;

void Task() {
  Metro metro{100};
  while (1) {
    if (metro.check()) {
      uint8_t cmd = random(8);
      if (cmd < 4) {
        servo.Position(0.0);
      } else {
        servo.Position(0.0);
      }

      {
        Threads::Scope lock{serial_mutex};
        Serial.println("thread");
      }
    }
  }
}

void setup() {
  SerialInitializer.init();
  SpiInitializer.init();
  CanFdInitializer.init(CANFD_BUS);
  servo.Stop();
  threads.addThread([] { Task(); });
}

void loop() {
  {
    Threads::Mutex lock{serial_mutex};
    Serial.println("loop");
  }

  threads.delay(1000);
}
