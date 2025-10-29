import serial
import time

def hex_to_percentage(hex_data):
    percentages = []
    # 每4个十六进制字符（2字节）解析为一个数值
    for i in range(0, len(hex_data), 4):
        high_byte = int(hex_data[i:i+2], 16)
        low_byte = int(hex_data[i+2:i+4], 16)
        value = (high_byte << 8) + low_byte
        percentages.append(value)
    return percentages

def read_serial_data(ser, expected_bytes):
    data = b''
    start_time = time.time()
    while len(data) < expected_bytes and (time.time() - start_time) < ser.timeout:
        if ser.in_waiting > 0:
            data += ser.read(ser.in_waiting)
    return data.hex()

def process_serial_commands():
    commands = [0xEE]  # 只使用0xEE命令读取编码器位置
    ser = serial.Serial(
        port='COM21',
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )
    
    try:
        for command in commands:
            ser.write(bytes([command]))
            time.sleep(0.01)  # 添加短暂延迟确保数据接收
            
            # 读取足够的数据：12路编码器 × 2字节/路 + 头尾标记
            # 假设头尾各1字节标记，所以总共需要26字节
            response = read_serial_data(ser, 26)
            
            # print(f"Raw response: {response}")  # 调试输出，显示原始数据
            
            if command == 0xEE and len(response) >= 26:
                # 去掉头尾标记（各1字节，即2个十六进制字符）
                clean_response = response[2:-2]
                # print(f"Cleaned response: {clean_response}")  # 调试输出
                
                percentages = hex_to_percentage(clean_response)
                return percentages
            else:
                print(f"Invalid response length: {len(response)}")
                return None
                
    except Exception as e:
        print(f"Serial communication error: {e}")
        return None
    finally:
        ser.close()

def restrict_range(value, lower=-2, upper=2):
    """Restricts a number within the range [-2, 2]."""
    return max(lower, min(value, upper))

def map_to_motor(value, in_min=0, in_max=9940, out_min=0, out_max=6.28):
    """Maps a value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def schedule_task(interval):
    try:
        while True:
            percentages = process_serial_commands()
            if percentages and len(percentages) >= 12:
                a1Pos = percentages[0:12]  # 获取全部12路编码器数据
                # print("positions:", a1Pos)
                
                # 如果需要映射到电机控制范围
                mapped_a1Pos = [map_to_motor(value) for value in a1Pos]
                # for i in [1, 2, 4]:
                #     mapped_a1Pos[i] = restrict_range(mapped_a1Pos[i])
                #     print("Mapped positions:", mapped_a1Pos[i])
                mapped_a1Pos[5] = restrict_range(mapped_a1Pos[5], 0, 6.28)
                print("Mapped positions:", mapped_a1Pos[5])


            else:
                print("Failed to read all 12 encoder positions")
            
            time.sleep(interval)

    except KeyboardInterrupt:
        print("Exiting...")

# Start the task scheduling every 0.1 seconds
schedule_task(0.1)