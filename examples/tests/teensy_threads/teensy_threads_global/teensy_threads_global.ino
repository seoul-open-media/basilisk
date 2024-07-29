// Expected output:
// 0 0 0 0 0 1 1 1 1 1 2 2 2 2 2 ...... 16 16 16 16 16
// 15 15 15 15 15 14 14 14 14 14 ...... 0 0 0 0 0 ............

#include <TeensyThreads.h>

volatile int data = 0;
Threads::Mutex mutex;

void increment() {
  while (1) {
    Threads::Scope lock{mutex};
    for (size_t i = 0; i < 16; i++) {
      data++;
      delay(250);
    }
  }
}

void decrement() {
  while (1) {
    Threads::Scope lock{mutex};
    for (size_t i = 0; i < 16; i++) {
      data--;
      delay(250);
    }
  }
}

void setup() {
  threads.addThread(increment);
  threads.addThread(decrement);
}

void loop() {
  Serial.println(data);
  delay(50);
}
