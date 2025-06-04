#include <Wire.h>

static constexpr uint8_t TCA9548A_ADDRESS = 0x70;
static constexpr uint8_t MT6701_ADDRESS   = 0x06;
static constexpr uint8_t ANGLE_REGISTER   = 0x03;
static constexpr int COUNTS_PER_REV       = 16384;
static constexpr float COUNTS_TO_DEGREES  = 360.0 / COUNTS_PER_REV;

void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

int readMT6701Angle() {
  Wire.beginTransmission(MT6701_ADDRESS);
  Wire.write(ANGLE_REGISTER);
  if (Wire.endTransmission(false) != 0) {
    return -1;
  }

  Wire.requestFrom(MT6701_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t angle_h = Wire.read();
    uint8_t angle_l = Wire.read();
    return (angle_h << 6) | (angle_l & 0x3F);
  }
  return -1;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
}

void loop() {
  Serial.print("[");  // Start JSON array

  for (uint8_t ch = 0; ch < 8; ch++) {
    selectMuxChannel(ch);
    delay(2);

    int angleCount = readMT6701Angle();
    float angleDeg = (angleCount >= 0) ? angleCount * COUNTS_TO_DEGREES : -1.0;

    Serial.print(angleDeg, 2);
    if (ch < 7) Serial.print(", ");
  }

  Serial.println("]");  // End JSON array
  delay(100); // Adjust as needed for sampling rate
}
