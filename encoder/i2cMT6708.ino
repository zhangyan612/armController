#include <Wire.h>

static constexpr uint8_t DEFAULT_ADDRESS = 0b0000110; // I2C address of the MT6701
static constexpr int COUNTS_PER_REVOLUTION = 16384;   // 14 bit encoder
static constexpr float COUNTS_TO_DEGREES = 360.0 / COUNTS_PER_REVOLUTION;

const int encoderAddress = DEFAULT_ADDRESS;
const int positionRegister = 0x03; // Starting register ANGLE_H

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud
  Wire.begin(); // Initialize I2C communication
  Wire.setClock(400000); // Set I2C clock speed to 400kHz
}

void loop() {
  int encoderValue = readEncoderPosition(); // Read encoder position
  if (encoderValue != -1) {
    float angle = encoderValue * COUNTS_TO_DEGREES; // Convert to degrees
    Serial.print("Encoder Position (Degrees): ");
    Serial.println(angle); // Print the encoder value to the serial monitor
  } else {
    Serial.println("Error reading encoder value.");
  }
  delay(1000); // Wait for a second before repeating
}

int readEncoderPosition() {
  Wire.beginTransmission(encoderAddress);
  Wire.write(positionRegister); // Point to the position register
  Wire.endTransmission(false); // End transmission, but keep the I2C bus active
  Wire.requestFrom(encoderAddress, 2); // Request 2 bytes from the encoder

  if (Wire.available() == 2) {
    int angle_h = Wire.read();
    int angle_l = Wire.read();
    int value = (angle_h << 6) | angle_l; // Combine the two bytes (14-bit value)
    return value;
  } else {
    return -1; // Error reading the encoder
  }
}
