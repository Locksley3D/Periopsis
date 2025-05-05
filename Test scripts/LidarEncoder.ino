const int CLOCK_PIN1 = 8;
const int DATA_PIN1 = 7;
const int BIT_COUNT = 12; // 12-bit encoder = 4096 steps

void setup() {
  pinMode(DATA_PIN1, INPUT);
  pinMode(CLOCK_PIN1, OUTPUT);
  digitalWrite(CLOCK_PIN1, HIGH); // idle state

  Serial.begin(115200);
  delay(50); // Let everything settle
}

void loop() {
  uint16_t raw = readEncoder(DATA_PIN1, CLOCK_PIN1, BIT_COUNT);

  float angle = raw * 360.0 / 4096.0;

  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print("  Angle: ");
  Serial.println(angle);

  delay(50);
}

uint16_t readEncoder(int dataPin, int clockPin, int bitCount) {
  uint16_t value = 0;

  for (int i = 0; i < bitCount; i++) {
    digitalWrite(clockPin, LOW);
    delayMicroseconds(5); // slower clock

    value <<= 1;
    if (digitalRead(dataPin)) {
      value |= 1;
    }

    digitalWrite(clockPin, HIGH);
    delayMicroseconds(5);
  }

  return value & 0x0FFF; // ensure only 12 bits
}
