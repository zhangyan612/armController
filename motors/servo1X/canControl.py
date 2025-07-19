import can
import time

# 配置参数
BITRATE = 500000  # CAN 波特率
NODE_ID = 1       # 电机节点 ID（站号）

# SDO 命令格式
SDO_READ = 0x40
SDO_WRITE = 0x20

# SDO 报文ID格式
SDO_TX_ID = 0x600 + NODE_ID
SDO_RX_ID = 0x580 + NODE_ID

# PDO 报文ID
TPDO1_ID = 0x180 + NODE_ID  # 实际位置、速度等数据
RPDO1_ID = 0x200 + NODE_ID  # 控制字、目标位置等

# 对象字典地址
OD_TEMPERATURE = (0x2301, 0x00)  # 温度
OD_ACTUAL_POSITION = (0x6064, 0x00)  # 实际位置
OD_ACTUAL_VELOCITY = (0x606C, 0x00)  # 实际速度
OD_CONTROL_WORD = (0x6040, 0x00)  # 控制字
OD_OPERATION_MODE = (0x6060, 0x00)  # 工作模式
OD_TARGET_POSITION = (0x607A, 0x00)  # 目标位置

# 工作模式定义
MODE_POSITION = 1  # 位置模式
MODE_SPEED = 3     # 速度模式
MODE_TORQUE = 4    # 力矩模式

# 控制字定义
CONTROL_WORD_SHUTDOWN = 0x0006  # 停止
CONTROL_WORD_ENABLE = 0x000F    # 使能
CONTROL_WORD_CLEAR_FAULT = 0x0086  # 清除错误

def send_sdo_read_request(bus, index, subindex):
    """
    发送SDO读取请求
    """
    data = [
        SDO_READ,
        index & 0xFF,
        (index >> 8) & 0xFF,
        subindex
    ] + [0] * 4
    msg = can.Message(arbitration_id=SDO_TX_ID, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"Sent SDO read request for index: 0x{index:04X}, subindex: 0x{subindex:02X}")

def read_sdo_response(bus, timeout=1.0):
    """
    等待SDO响应
    """
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == SDO_RX_ID:
            command = msg.data[0]
            index = msg.data[1] | (msg.data[2] << 8)
            subindex = msg.data[3]
            data = msg.data[4:]
            print(f"Received SDO response: Index 0x{index:04X}, Subindex 0x{subindex:02X}, Data: {data}")
            return data
    print("Timeout waiting for SDO response.")
    return None

def send_sdo_write_command(bus, index, subindex, data_bytes):
    """
    发送SDO写入命令
    """
    data = [
        SDO_WRITE,
        index & 0xFF,
        (index >> 8) & 0xFF,
        subindex
    ]
    data += data_bytes + [0] * (4 - len(data_bytes))  # 补齐4字节
    msg = can.Message(arbitration_id=SDO_TX_ID, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"Sent SDO write command: Index 0x{index:04X}, Subindex 0x{subindex:02X}, Data: {data}")

def read_motor_data(bus):
    """
    读取电机的温度、位置、速度
    """
    # 读取温度
    send_sdo_read_request(bus, OD_TEMPERATURE[0], OD_TEMPERATURE[1])
    temp_data = read_sdo_response(bus)
    if temp_data:
        temperature = int.from_bytes(temp_data[:2], byteorder='little', signed=True)
        print(f"Motor Temperature: {temperature / 10.0} °C")

    # 读取实际位置
    send_sdo_read_request(bus, OD_ACTUAL_POSITION[0], OD_ACTUAL_POSITION[1])
    pos_data = read_sdo_response(bus)
    if pos_data:
        position = int.from_bytes(pos_data[:4], byteorder='little', signed=True)
        print(f"Motor Position: {position} counts")

    # 读取实际速度
    send_sdo_read_request(bus, OD_ACTUAL_VELOCITY[0], OD_ACTUAL_VELOCITY[1])
    vel_data = read_sdo_response(bus)
    if vel_data:
        velocity = int.from_bytes(vel_data[:4], byteorder='little', signed=True)
        print(f"Motor Velocity: {velocity / 1000.0} RPM")

def set_position_mode(bus):
    """
    设置电机为位置控制模式
    """
    send_sdo_write_command(bus, OD_OPERATION_MODE[0], OD_OPERATION_MODE[1],
                           MODE_POSITION.to_bytes(2, byteorder='little'))
    print("Set operation mode to position control")

def send_target_position(bus, target_position):
    """
    发送目标位置（RPDO1）
    """
    data = [
        MODE_POSITION & 0xFF,
        (MODE_POSITION >> 8) & 0xFF,
        CONTROL_WORD_ENABLE & 0xFF,
        (CONTROL_WORD_ENABLE >> 8) & 0xFF,
        target_position & 0xFF,
        (target_position >> 8) & 0xFF,
        (target_position >> 16) & 0xFF,
        (target_position >> 24) & 0xFF
    ]
    msg = can.Message(arbitration_id=RPDO1_ID, data=data, is_extended_id=False)
    bus.send(msg)
    print(f"Sent target position: {target_position} counts")

def main():
    # 检测可用的 CAN 接口
    available_configs = can.detect_available_configs()
    print("Available CAN configurations:", available_configs)

    # 使用第一个可用的 pcan 接口配置
    for config in available_configs:
        if config['interface'] == 'pcan':
            try:
                # 初始化 CAN 总线
                bus = can.interface.Bus(**config, bitrate=BITRATE)
                print(f"Successfully initialized CAN bus on {config['channel']} with interface {config['interface']}")

                # 读取电机信息
                read_motor_data(bus)

                # 设置为位置控制模式
                set_position_mode(bus)

                # 发送目标位置
                target_pos = 1000  # 例如：100000 counts
                send_target_position(bus, target_pos)

                # 等待电机响应
                time.sleep(2)

                # 关闭总线
                bus.shutdown()
                print("CAN bus closed.")
                break

            except Exception as e:
                print(f"Failed to initialize CAN bus: {e}")
            break
    else:
        print("No suitable PCAN interface found.")

if __name__ == "__main__":
    main()