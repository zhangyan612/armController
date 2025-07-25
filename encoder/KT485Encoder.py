import serial
import struct
import time
import crcmod

# CRC16 校验函数
crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)

# 配置参数
PORT = 'COM5'  # 修改为你使用的串口号
BAUDRATE = 115200
PARITY = serial.PARITY_NONE
BYTESIZE = serial.EIGHTBITS
STOPBITS = serial.STOPBITS_ONE
SLAVE_ID = 1  # 默认从机地址为1，可在运行中修改

# 打开端口
ser = serial.Serial(PORT, BAUDRATE, bytesize=BYTESIZE, parity=PARITY, stopbits=STOPBITS, timeout=1)


# 构造 Modbus-RTU 请求帧
def modbus_request(slave_id, function_code, start_reg, count=1):
    if function_code == 0x03:
        # 读输入寄存器
        msg = struct.pack('>BBHH', slave_id, function_code, start_reg, count)
    elif function_code == 0x06:
        # 写单个寄存器
        value = count  # 这里 count 表示写入的值
        msg = struct.pack('>BBHH', slave_id, function_code, start_reg, value)
    else:
        raise ValueError("不支持的功能码")

    crc = crc16(msg)
    request = msg + struct.pack('<H', crc)
    return request


# 发送 Modbus 请求并返回响应
def send_request(request):
    ser.write(request)
    time.sleep(0.05)  # 等待响应
    response = ser.read(ser.in_waiting or 0)
    return response


# 读取单圈位置（地址13）
def read_single_position():
    request = modbus_request(SLAVE_ID, 0x03, 13)
    response = send_request(request)

    if len(response) >= 5:
        value = struct.unpack('>H', response[3:5])[0]
        return value
    else:
        print("读取单圈位置失败")
        return None


# 读取多圈位置（地址15-16）
def read_multi_position():
    request = modbus_request(SLAVE_ID, 0x03, 15, count=2)
    response = send_request(request)

    if len(response) >= 7:
        low = struct.unpack('>H', response[5:7])[0]
        high = struct.unpack('>H', response[3:5])[0]
        return (high << 16) | low
    else:
        print("读取多圈位置失败")
        return None

# 读取单圈角度（地址22）
def read_single_angle():
    pos = read_register(22)
    if pos is not None:
        return pos / 100.0  # 单位 0.01°
    return None


# 读取多圈角度（地址20-21）
def read_multi_angle():
    request = modbus_request(SLAVE_ID, 0x03, 20, count=2)
    response = send_request(request)

    if len(response) >= 7:
        low = struct.unpack('>H', response[5:7])[0]
        high = struct.unpack('>H', response[3:5])[0]
        raw = (high << 16) | low
        return raw / 100.0
    else:
        print("读取多圈角度失败")
        return None


# 读取转速（地址18）
def read_speed():
    pos = read_register(18)
    if pos is not None:
        return pos / 10.0  # 单位 0.1转
    return None


# 通用读寄存器函数
def read_register(address):
    request = modbus_request(SLAVE_ID, 0x03, address)
    response = send_request(request)

    if len(response) >= 5:
        return struct.unpack('>H', response[3:5])[0]
    else:
        print(f"读取寄存器 {address} 失败")
        return None


# 设置模块地址（地址1）
def set_slave_address(new_address):
    global SLAVE_ID

    if new_address in [0, 46, 255]:
        print("地址不能设置为 0, 46 或 255")
        return False

    request = modbus_request(SLAVE_ID, 0x06, 1, new_address)
    response = send_request(request)

    if len(response) >= 8 and response[1] == 0x06:
        SLAVE_ID = new_address
        print(f"地址已更改为 {new_address}")
        return True
    else:
        print("设置地址失败")
        return False


# 设置波特率（地址2）
def set_baud_rate(baud_value):
    """
    0: 115200
    2: 38400
    3: 19200
    4: 9600
    """
    if baud_value not in [0, 2, 3, 4]:
        print("波特率设置错误")
        return False

    request = modbus_request(SLAVE_ID, 0x06, 2, baud_value)
    response = send_request(request)

    if len(response) >= 8 and response[1] == 0x06:
        print(f"波特率已设置为 {baud_value}")
        return True
    else:
        print("设置波特率失败")
        return False


# 启用/禁用主动输出（地址4）
def enable_active_output(enable):
    value = 1 if enable else 0
    request = modbus_request(SLAVE_ID, 0x06, 4, value)
    response = send_request(request)

    if len(response) >= 8 and response[1] == 0x06:
        print("主动输出已启用" if enable else "主动输出已禁用")
        return True
    else:
        print("设置主动输出失败")
        return False


# 获取模块地址（特殊命令）
def get_slave_address():
    special_cmd = bytes.fromhex('FF 03 00 01 00 01 C0 14')
    ser.write(special_cmd)
    time.sleep(0.1)
    res = ser.read(8)

    if res:
        print("收到响应:", res.hex())
        if len(res) >= 5:
            address = res[3]
            print(f"模块地址为: {address}")
            return address
    return None

def set_position_zero():
    """
    向寄存器地址 11 写入值 1，触发位置置零操作。
    使用 Modbus 功能码 0x06（写单个寄存器）
    """
    request = modbus_request(SLAVE_ID, 0x06, 11, 1)  # 地址11，写入值1
    response = send_request(request)

    if response and len(response) >= 6 and response[1] == 0x06:
        print("位置置零命令已发送成功")
        return True
    else:
        print("位置置零命令发送失败，未收到有效响应")
        return False


# 接收主动输出帧（串口监听）
def listen_for_active_output():
    print("开始监听主动输出数据（按 Ctrl+C 停止）...")
    buffer = bytearray()

    while True:
        data = ser.read(100)
        if data:
            buffer.extend(data)

            while len(buffer) >= 5:
                if buffer[0] == 0xFF and buffer[1] == 0x81:
                    high = buffer[2]
                    low = buffer[3]
                    checksum = buffer[4]
                    angle_raw = (high << 8) | low
                    angle_deg = angle_raw / 65536.0 * 360

                    # ✅ 直接输出角度，跳过校验
                    print(f"主动输出角度: {angle_deg:.2f}°")

                    # 移除已处理的帧
                    buffer = buffer[5:]
                else:
                    buffer.pop(0)  # 丢弃无效字节
        time.sleep(0.01)

# 示例主程序
if __name__ == "__main__":
    try:
        print("单圈位置:", read_single_position())
        # print("多圈位置:", read_multi_position())
        # print("单圈角度:", read_single_angle())
        # print("多圈角度:", read_multi_angle())
        # print("转速:", read_speed())

        # success = set_position_zero()
        # if success:
        #     print("编码器位置已置零")
        # else:
        #     print("置零失败，请检查连接或地址配置")
        # time.sleep(1)
        # print("单圈位置:", read_single_position())
        # print("多圈位置:", read_multi_position())

        # # 示例：获取模块地址
        # address=get_slave_address()
        # print(f"当前模块地址: {address}")
        # 示例：设置新地址
        # set_slave_address(2)

        # 示例：设置波特率
        # set_baud_rate(3)  # 19200

        # 示例：启用主动输出
        # enable_active_output(False)

        # 示例：监听主动输出
        # listen_for_active_output()

    except KeyboardInterrupt:
        print("用户中断")

    finally:
        ser.close()
        print("串口已关闭")