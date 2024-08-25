#include "../basilisk.h"

void SerialPlotterReplySender() {
  Serial.print("rho_l:");
  Serial.print(basilisk.l_.GetQRpl().position);
  Serial.print(",");
  Serial.print("rho_r:");
  Serial.println(basilisk.r_.GetQRpl().position);
}
