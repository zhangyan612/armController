 #include <SPI.h>

// 硬件引脚定义
const int CS_PIN = 10;    // 片选引脚
const int LED_PIN = 13;   // 板载LED

// MA730寄存器地址
#define REG_ANGLE 0x0000
#define REG_STATUS 0x0B

// 全局变量
volatile int64_t total_angle = 0;  // 累计角度值
volatile uint16_t last_angle = 0;  // 上一次的角度值
uint8_t first_read = 2;            // 初始化标记

// 函数声明
uint16_t readMA730(uint16_t command);
uint16_t readAngle();
void initMA730();
float getAngleRadians();
void updateAngle();

void setup() {
  // 初始化引脚
  pinMode(CS_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  
  // 初始化串口
  Serial.begin(115200);
  
  // 初始化SPI
  SPI.begin();
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE3)); // 10MHz, 模式3
  
  // 初始化编码器
  initMA730();
  
  // 初始读取
  readAngle();
  delay(100);
  readAngle();
  
  Serial.println("System Ready");
  Serial.println("Angle (radians)");
}

void loop() {
  // 读取角度并更新累计值
  updateAngle();
  
  // 获取当前角度（弧度）
  float current_angle = getAngleRadians();
  
  // 输出到串口
  Serial.println(current_angle, 5);
  
  // 控制LED闪烁
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  
  // 延迟控制采样率
  delay(10);
}

// 读取MA730寄存器
uint16_t readMA730(uint16_t command) {
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);
  
  // 发送命令并接收数据
  uint16_t response = SPI.transfer16(command);
  
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  
  return response;
}

// 读取角度值
uint16_t readAngle() {
  return readMA730(REG_ANGLE) >> 2; // 14位有效数据
}

// 初始化MA730
void initMA730() {
  // 简单的初始化序列
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(10);
  digitalWrite(CS_PIN, HIGH);
  delay(10);
}

// 获取当前角度（弧度）
float getAngleRadians() {
  // 14位分辨率：16384计数/圈
  return (total_angle * 2.0 * PI) / 16384.0;
}

// 更新角度累计值
void updateAngle() {
  uint16_t current_angle = readAngle();
  
  if (first_read) {
    // 初始化阶段
    last_angle = current_angle;
    total_angle = (int32_t)current_angle;
    first_read--;
    return;
  }
  
  // 计算角度变化（处理0-16383边界）
  int16_t delta = current_angle - last_angle;
  
  // 处理过零情况
  if (delta > 8192) {
    delta -= 16384;  // 逆时针过零
  } else if (delta < -8192) {
    delta += 16384;  // 顺时针过零
  }
  
  // 更新累计角度
  total_angle += delta;
  last_angle = current_angle;
}