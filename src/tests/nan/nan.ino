void print_bin(float* f) {
  const auto bytes = *reinterpret_cast<uint32_t*>(f);

  for (uint8_t i = 0; i < 32; i++) {
    Serial.print((bytes & (1 << (31 - i))) ? "1" : "0");
  }
  Serial.println();
}

void print_bin(double* d) {
  const auto bytes = *reinterpret_cast<uint64_t*>(d);

  for (uint8_t i = 0; i < 64; i++) {
    Serial.print((bytes & (1ULL << (63 - i))) ? "1" : "0");
  }
  Serial.println();
}

#define NaN (0.0 / 0.0)

void setup() {
  Serial.begin(9600);

  float nanf = NaN;
  Serial.print("float NaN: ");
  print_bin(&nanf);

  double nand = NaN;
  Serial.print("double NaN: ");
  print_bin(&nand);

  float nand2f = static_cast<float>(nand);
  Serial.print("static_cast<float>(double) NaN: ");
  print_bin(&nand2f);

  double nanf2d = static_cast<double>(nanf);
  Serial.print("static_cast<double>(float) NaN: ");
  print_bin(&nanf2d);
}

void loop() {}
