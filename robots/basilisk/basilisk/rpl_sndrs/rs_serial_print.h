#include "../basilisk.h"

void SerialPrintReplySender() {
  basilisk.CommandBoth([](Servo& s) { s.Print(); });
}
