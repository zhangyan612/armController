import serial
import time

# 配置串口参数
port = 'COM35'
baudrate = 921600

try:
    # 创建串口连接
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Connected to {ser.name}")
    
    # 持续读取数据
    while True:
        if ser.in_waiting > 0:
            # 读取一行数据（根据实际协议调整）
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            if data:
                print(data)
        time.sleep(0.01)  # 短暂延迟以减少CPU占用

except serial.SerialException as e:
    print(f"Serial connection error: {e}")
except KeyboardInterrupt:
    print("Program interrupted by user")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed")