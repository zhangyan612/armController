import serial
import struct
import time

# 配置串口
ser = serial.Serial(
    port='COM6',     # 替换为您的实际串口
    baudrate=115200, # 波特率
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1        # 读/写超时时间
)

# Modbus CRC16计算函数
def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

# 读取单圈绝对位置 (Modbus寄存器地址13)
def read_position():
    # 构造Modbus请求帧: [从机地址, 功能码, 寄存器地址高, 寄存器地址低, 寄存器数量高, 寄存器数量低, CRC低, CRC高]
    slave_id = 1  # 默认从机地址
    request = struct.pack('>BBHH', slave_id, 0x03, 13, 1)
    crc = crc16(request)
    request += struct.pack('<H', crc)  # 小端序添加CRC
    
    # 发送请求
    ser.write(request)
    time.sleep(0.05)  # 等待设备响应
    
    # 读取响应 (应返回7字节)
    response = ser.read(7)
    
    if len(response) == 7:
        # 验证CRC
        received_crc = struct.unpack('<H', response[-2:])[0]
        calculated_crc = crc16(response[:-2])
        
        if received_crc == calculated_crc:
            # 解析位置值 (大端序)
            value = struct.unpack('>H', response[3:5])[0]
            return value
    return None

# 主函数
if __name__ == "__main__":
    try:
        position = read_position()
        if position is not None:
            print(f"单圈位置值: {position}")
        else:
            print("读取位置失败")
    finally:
        ser.close()