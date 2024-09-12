// LPS
uint8_t lps_vals[4];

// IMU
#define SBUF_SIZE 64
char sbuf[SBUF_SIZE];
signed int sbuf_cnt = 0;

// Foot switches
int foot_switch_l = 23;
int foot_switch_r = 29;

int EBimuAsciiParser(float* item, int number_of_item) {
  int n, i;
  int rbytes;
  char* addr;
  int result = 0;

  rbytes = Serial2.available();
  for (n = 0; n < rbytes; n++) {
    sbuf[sbuf_cnt] = Serial2.read();
    if (sbuf[sbuf_cnt] == 0x0a) {
      addr = strtok(sbuf, ",");
      for (i = 0; i < number_of_item; i++) {
        item[i] = atof(addr);
        addr = strtok(NULL, ",");
      }

      result = 1;
    } else if (sbuf[sbuf_cnt] == '*') {
      sbuf_cnt = -1;
    }

    sbuf_cnt++;
    if (sbuf_cnt >= SBUF_SIZE) sbuf_cnt = 0;
  }

  return result;
}

void setup() {
  Serial.begin(9600);
  Serial6.begin(9600);
  Serial2.begin(57600);

  pinMode(foot_switch_l, INPUT);
  pinMode(foot_switch_r, INPUT);
}

void loop() {
  if (Serial6.available() >= 6) {
    Serial.print(F("LPS: 6 or more bytes received. "));

    if (Serial6.read() == 255 && Serial6.read() == 2) {
      Serial.print(F("Two start bytes matched. Last 4 bytes: "));
      for (auto i = 0; i < 4; i++) {
        lps_vals[i] = Serial6.read();

        Serial.print(lps_vals[i]);
        if (i != 3) Serial.print(F(", "));
      }
    } else {
      Serial.print(F("Wrong start bytes. Flushing LPS Serial"));

      while (Serial6.available()) Serial6.read();
    }
    Serial.println();
  }

  {
    float euler[3];

    if (EBimuAsciiParser(euler, 3)) {
      Serial.print(euler[0]);
      Serial.print(" ");
      Serial.print(euler[1]);
      Serial.print(" ");
      Serial.print(euler[2]);
      Serial.println();
    }
  }

  {
    Serial.print(F("Foot switches: "));
    Serial.print(digitalRead(foot_switch_l));
    Serial.print(", ");
    Serial.println(digitalRead(foot_switch_r));
  }
}
