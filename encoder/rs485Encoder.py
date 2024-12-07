import serial
import time

# 配置串口
ser = serial.Serial(
    port='COM17',     # 替换为您的实际串口
    baudrate=115200, # 波特率
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1        # 读/写超时时间
)

# 计算 CRC 校验码
def calculate_crc(cf, sf, df0, df1):
    return cf ^ sf ^ df0 ^ df1

# 发送指令帧
def send_command(command, repeat=1):
    for _ in range(repeat):
        ser.write(command)
        time.sleep(0.0008) # 时间间隔 >= 600微秒

# 读取单圈绝对位置
def read_position():
    send_command(b'\x02')
    response = ser.read(5)  # 读取 5 个字节的响应数据
    print(response)
    if len(response) == 5:
        cf = response[0]
        sf = response[1]
        df0 = response[2]
        df1 = response[3]
        crc = response[4]
        calculated_crc = calculate_crc(cf, sf, df0, df1)
        if crc == calculated_crc:
            angle = df0 + (df1 << 8)
            status = sf  # 运行状态
            return angle, status, response
    return None, None, response

# 校准操作
def calibrate():
    # 保持电机匀速开环运转，转速6rpm~500rpm
    # 连续发送指令帧0xBA
    for _ in range(10):
        send_command(b'\xBA')
        response = ser.read(5)
        if response:
            cf = response[0]
            sf = response[1]
            df0 = response[2]
            df1 = response[3]
            crc = response[4]
            calculated_crc = calculate_crc(cf, sf, df0, df1)
            if crc == calculated_crc:
                angle = df0 + (df1 << 8)
                status = sf
                print(f"校准中，角度: {angle}, 运行状态: {status}")
            time.sleep(0.0007) # 保证两次发送间隔大于600微秒

    # 校准初始化
    print("校准初始化...")
    time.sleep(1)  # 初始化时间约为100ms

    # 连续发送指令帧0x02，直到有数据返回并且状态帧为0x08
    while True:
        angle, status, response = read_position()
        if status is not None:
            print(f"角度: {angle}, 运行状态: {status}")
            if status == 0x08:
                print("校准结束")
                break

# 单圈绝对位置归零
def reset_position():
    for _ in range(10):
        send_command(b'\xC2')
        response = ser.read(5)
        if response:
            cf = response[0]
            sf = response[1]
            df0 = response[2]
            df1 = response[3]
            crc = response[4]
            calculated_crc = calculate_crc(cf, sf, df0, df1)
            if crc == calculated_crc:
                angle = df0 + (df1 << 8)
                status = sf
                print(f"清零中，角度: {angle}, 运行状态: {status}")
            time.sleep(0.0007) # 保证两次发送间隔大于600微秒

    print("清零操作完成")

# 持续输出位置信号并每秒打印一次
def continuous_output():
    try:
        while True:
            angle, status, response = read_position()
            if angle is not None and status is not None:
                print(f"角度: {angle}, 运行状态: {status}")
            time.sleep(1)  # 每秒打印一次
    except KeyboardInterrupt:
        print("停止输出位置信号")

# 主函数
if __name__ == "__main__":
    # 示例操作
    # reset_position()
    # calibrate()
    continuous_output()

# 使用完毕关闭串口
ser.close()


#success