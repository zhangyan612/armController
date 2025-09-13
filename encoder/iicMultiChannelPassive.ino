#include <Wire.h>

// TCA9548A 地址
static constexpr uint8_t TCA9548A_ADDRESSES[] = {0x70, 0x71, 0x72};
static constexpr uint8_t NUM_TCA = sizeof(TCA9548A_ADDRESSES) / sizeof(TCA9548A_ADDRESSES[0]);

// MT6701 参数
static constexpr uint8_t MT6701_ADDRESS = 0x06;
static constexpr uint8_t ANGLE_REGISTER = 0x03;
static constexpr int COUNTS_PER_REV = 16384;
static constexpr float COUNTS_TO_DEGREES = 360.0 / COUNTS_PER_REV;

// ----------------- I2C 多路复用器 -----------------
void disableAllTCA() {
  for (uint8_t i = 0; i < NUM_TCA; i++) {
    Wire.beginTransmission(TCA9548A_ADDRESSES[i]);
    Wire.write(0x00);
    Wire.endTransmission();
  }
}

void selectMuxChannel(uint8_t tcaAddr, uint8_t channel) {
  if (channel > 7) return;
  disableAllTCA();
  Wire.beginTransmission(tcaAddr);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(300);
}

// ----------------- MT6701 读取 -----------------
int readMT6701Angle() {
  Wire.beginTransmission(MT6701_ADDRESS);
  Wire.write(ANGLE_REGISTER);
  if (Wire.endTransmission(false) != 0) return -1;
  Wire.requestFrom(MT6701_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t angle_h = Wire.read();
    uint8_t angle_l = Wire.read();
    return (angle_h << 6) | (angle_l & 0x3F);
  }
  return -1;
}

// ----------------- 解析单条批量命令 -----------------
void processBatchCommand(String cmd) {
  cmd.trim();
  if (cmd.length() < 3 || cmd.charAt(0) != 'B') {
    Serial.println("{\"error\":\"invalid command\"}");
    return;
  }

  // 拆分逗号
  int lastIndex = 2;
  int count = 0;
  int boards[16]; // 最多查询 8 对
  int ports[16];

  while (lastIndex < cmd.length() && count < 16) {
    int comma = cmd.indexOf(',', lastIndex);
    String token;
    if (comma < 0) token = cmd.substring(lastIndex);
    else token = cmd.substring(lastIndex, comma);

    if (count % 2 == 0) boards[count / 2] = token.toInt();
    else ports[count / 2] = token.toInt();

    count++;
    if (comma < 0) break;
    lastIndex = comma + 1;
  }

  int pairCount = count / 2;
  if (pairCount == 0) {
    Serial.println("{\"error\":\"no valid pairs\"}");
    return;
  }

  // 输出 JSON 数组
  Serial.print("[");
  for (int i = 0; i < pairCount; i++) {
    int boardId = boards[i];
    int port = ports[i];
    uint8_t tcaAddr = 0;
    if (boardId == 70) tcaAddr = 0x70;
    else if (boardId == 71) tcaAddr = 0x71;
    else if (boardId == 72) tcaAddr = 0x72;
    else {
      Serial.print("{\"board\":");
      Serial.print(boardId);
      Serial.print(",\"port\":");
      Serial.print(port);
      Serial.print(",\"angle\":-1}");
      if (i < pairCount - 1) Serial.print(",");
      continue;
    }

    uint8_t ch = port - 1;
    selectMuxChannel(tcaAddr, ch);
    delay(5);

    int angleCount = readMT6701Angle();
    float angleDeg = (angleCount >= 0) ? angleCount * COUNTS_TO_DEGREES : -1.0;

    Serial.print("{\"board\":");
    Serial.print(boardId);
    Serial.print(",\"port\":");
    Serial.print(port);
    Serial.print(",\"angle\":");
    Serial.print(angleDeg, 2);
    Serial.print("}");
    if (i < pairCount - 1) Serial.print(",");
  }
  Serial.println("]");
}

// ----------------- Arduino 主程序 -----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("Ready");  // 启动完成
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd.startsWith("B")) processBatchCommand(cmd);
  }
}
