import serial
import time

# 配置串口参数
ser = serial.Serial(
    port='COM3',          # 根据实际情况修改串口号
    baudrate=115200,       # 波特率
    bytesize=8,           # 数据位
    parity='N',           # 校验位
    stopbits=1,           # 停止位
    timeout=1             # 超时时间
)

# 要发送的数据（示例：十六进制数据）
data_to_send = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A])

try:
    ser.write(data_to_send)
    print("Sent:", data_to_send.hex())
    # 如果需要接收回复，可以接着读取
    response = ser.read(10)  # 读取10个字节，根据实际情况调整
    print("Received:", response.hex())
except Exception as e:
    print("Error:", e)
finally:
    ser.close()