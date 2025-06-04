#include <Wire.h>

// === Configuration ===
static constexpr uint8_t TCA9548A_ADDRESS = 0x70;     // Default address of TCA9548A
static constexpr uint8_t MT6701_ADDRESS   = 0x06;     // I2C address of MT6701 (binary 0000110)
static constexpr uint8_t ANGLE_REGISTER   = 0x03;     // MT6701 ANGLE_H register
static constexpr int COUNTS_PER_REV       = 16384;    // 14-bit resolution
static constexpr float COUNTS_TO_DEGREES  = 360.0 / COUNTS_PER_REV;

// === Function: Select TCA9548A Channel ===
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDRESS);
  Wire.write(1 << channel);  // Only enable one channel at a time
  Wire.endTransmission();
}

// === Function: Read MT6701 14-bit Angle ===
int readMT6701Angle() {
  Wire.beginTransmission(MT6701_ADDRESS);
  Wire.write(ANGLE_REGISTER);           // Point to ANGLE_H
  if (Wire.endTransmission(false) != 0) {
    return -1;  // Transmission error
  }

  Wire.requestFrom(MT6701_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t angle_h = Wire.read();       // ANGLE_H (8 bits)
    uint8_t angle_l = Wire.read();       // ANGLE_L (only lower 6 bits used)
    int angle = (angle_h << 6) | (angle_l & 0x3F);  // Combine 8+6 bits
    return angle;
  }
  return -1; // Read error
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Optional: Faster I2C communication
  Serial.println("Starting MT6701 + TCA9548A multi-encoder reader...");
}

// === Main Loop ===
void loop() {
  for (uint8_t ch = 0; ch < 8; ch++) {
    selectMuxChannel(ch);
    delay(2); // Allow some time for switching

    int angleCount = readMT6701Angle();
    Serial.print("Encoder ");
    Serial.print(ch);
    if (angleCount >= 0) {
      float angleDeg = angleCount * COUNTS_TO_DEGREES;
      Serial.print(": ");
      Serial.print(angleCount);
      Serial.print(" counts (");
      Serial.print(angleDeg, 2);
      Serial.println(" deg)");
    } else {
      Serial.println(": Error reading data");
    }
  }

  Serial.println("----");
  delay(500);
}
