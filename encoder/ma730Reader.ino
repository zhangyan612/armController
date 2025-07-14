#include <SPI.h>

// 硬件引脚定义
const int CS_PIN = 10;    // 片选引脚
const int LED_PIN = 13;   // 板载LED

// MA730寄存器地址
#define REG_ANGLE 0x0000  // 角度寄存器

// 全局变量
volatile uint16_t raw_angle = 0;       // 当前原始角度值
volatile int32_t total_count = 0;      // 累计角度计数值（多圈）
volatile uint16_t last_angle = 0;      // 上一次的角度值
bool initialized = false;              // 初始化标志

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
  
  // 初始读取
  readAngle();
  delay(10);
  readAngle();  // 第二次读取完成初始化
  
  Serial.println("MA730 Encoder Reader");
  Serial.println("Raw Angle | Total Count");
}

void loop() {
  // 读取并更新角度值
  updateAngle();
  
  // 输出原始角度和累计计数值
  Serial.print(raw_angle);
  Serial.print(" | ");
  Serial.println(total_count);
  
  // 控制LED闪烁
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  
  // 延迟控制采样率
  delay(10);
}

// 读取MA730角度寄存器
uint16_t readAngle() {
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(1);
  
  // 发送角度寄存器读取命令 (0x0000)
  uint16_t response = SPI.transfer16(0x0000);
  
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  
  // 返回14位有效数据 (高14位)
  return response >> 2;
}

// 更新角度值并处理多圈计数
void updateAngle() {
  uint16_t current_angle = readAngle();
  
  if (!initialized) {
    // 初始化阶段
    last_angle = current_angle;
    total_count = (int32_t)current_angle;
    initialized = true;
    return;
  }
  
  // 计算角度变化（处理0-16383边界）
  int16_t delta = current_angle - last_angle;
  
  // 处理过零情况（1/4圈阈值）
  if (delta > 4096) {
    delta -= 16384;  // 逆时针过零
  } else if (delta < -4096) {
    delta += 16384;  // 顺时针过零
  }
  
  // 更新累计计数值
  total_count += delta;
  
  // 更新当前值
  raw_angle = current_angle;
  last_angle = current_angle;
}