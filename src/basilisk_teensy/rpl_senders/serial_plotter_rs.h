#pragma once

void SerialPlotterReplySender(Basilisk& b) {
  Serial.print("rho_l:");
  Serial.print(b.l_.GetQRpl().position);
  Serial.print(",");
  Serial.print("rho_r:");
  Serial.println(b.r_.GetQRpl().position);
}
