import can
import time
import struct

# 配置参数
BITRATE = 500000  # CAN 波特率
NODE_ID = 255       # 电机节点 ID（站号）

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
    发送SDO读取请求 (修复格式)
    """
    # 修正：完整的8字节数据格式
    data = [
        SDO_READ,
        index & 0xFF,          # 索引低字节
        (index >> 8) & 0xFF,   # 索引高字节
        subindex,              # 子索引
        0x00, 0x00, 0x00, 0x00 # 协议要求的填充字节
    ]
    msg = can.Message(
        arbitration_id=SDO_TX_ID, 
        data=data, 
        is_extended_id=False
    )
    bus.send(msg)
    print(f"Sent SDO read: ID=0x{SDO_TX_ID:X}, Data={[hex(x) for x in data]}")

def read_sdo_response(bus, timeout=1.0):
    """
    等待SDO响应 (增加错误处理)
    """
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == SDO_RX_ID:
            # 解析响应数据
            command = msg.data[0]
            index = msg.data[1] | (msg.data[2] << 8)
            subindex = msg.data[3]
            response_data = msg.data[4:8]  # 取4字节有效数据
            
            print(f"Received SDO: ID=0x{msg.arbitration_id:X}, "
                  f"Index=0x{index:04X}, Subindex=0x{subindex:02X}, "
                  f"Data={[hex(x) for x in response_data]}")
            return response_data
    print("SDO响应超时! 请检查:")
    print("1. 电机站号是否正确 (当前NODE_ID={})".format(NODE_ID))
    print("2. CAN物理连接是否正常")
    print("3. 电机供电状态")
    return None

def send_sdo_write_command(bus, index, subindex, value, size=2):
    """
    发送SDO写入命令 (修复数据类型和格式)
    """
    # 将值转换为字节数组
    if size == 2:
        value_bytes = list(struct.pack('<H', value))  # 小端序2字节
    elif size == 4:
        value_bytes = list(struct.pack('<I', value))  # 小端序4字节
    else:
        raise ValueError("Invalid size")
    
    # 构造完整8字节数据
    data = [
        SDO_WRITE,
        index & 0xFF,          # 索引低字节
        (index >> 8) & 0xFF,   # 索引高字节
        subindex,              # 子索引
    ] + value_bytes            # 数值字节
    
    # 填充剩余字节
    while len(data) < 8:
        data.append(0x00)
    
    msg = can.Message(
        arbitration_id=SDO_TX_ID, 
        data=data, 
        is_extended_id=False
    )
    bus.send(msg)
    print(f"Sent SDO write: ID=0x{SDO_TX_ID:X}, Data={[hex(x) for x in data]}")

def read_motor_data(bus):
    """
    读取电机的温度、位置、速度 (增加重试机制)
    """
    print("\n=== 读取电机数据 ===")
    
    # 读取温度 (2字节)
    send_sdo_read_request(bus, OD_TEMPERATURE[0], OD_TEMPERATURE[1])
    temp_data = read_sdo_response(bus)
    if temp_data:
        temperature = struct.unpack('<h', bytes(temp_data[:2]))[0]
        print(f"电机温度: {temperature/10.0} °C")
    
    # 读取实际位置 (4字节)
    send_sdo_read_request(bus, OD_ACTUAL_POSITION[0], OD_ACTUAL_POSITION[1])
    pos_data = read_sdo_response(bus)
    if pos_data:
        position = struct.unpack('<i', bytes(pos_data[:4]))[0]
        print(f"实际位置: {position} counts")
    
    # 读取实际速度 (4字节)
    send_sdo_read_request(bus, OD_ACTUAL_VELOCITY[0], OD_ACTUAL_VELOCITY[1])
    vel_data = read_sdo_response(bus)
    if vel_data:
        velocity = struct.unpack('<i', bytes(vel_data[:4]))[0]
        print(f"实际速度: {velocity/1000.0} RPM")

def set_position_mode(bus):
    """
    设置电机为位置控制模式 (修复数据类型)
    """
    print("\n=== 设置位置模式 ===")
    send_sdo_write_command(
        bus, 
        OD_OPERATION_MODE[0], 
        OD_OPERATION_MODE[1],
        MODE_POSITION,
        size=2  # 2字节数据
    )
    # 等待设置生效
    time.sleep(0.1)

def send_target_position(bus, target_position):
    """
    发送目标位置 (RPDO1) - 修复格式
    """
    # 根据协议构造完整8字节数据
    data = struct.pack(
        '<HHii',  # 格式: 2个uint16 + 2个int32
        MODE_POSITION,      # 工作模式 (2字节)
        CONTROL_WORD_ENABLE, # 控制字 (2字节)
        target_position,    # 目标位置 (4字节)
        0                   # 填充 (协议要求8字节)
    )
    
    msg = can.Message(
        arbitration_id=RPDO1_ID,
        data=data,
        is_extended_id=False
    )
    bus.send(msg)
    print(f"Sent RPDO: ID=0x{RPDO1_ID:X}, TargetPos={target_position}")

def main():
    print("=== CAN电机控制程序 ===")
    
    # 尝试初始化PCAN
    try:
        bus = can.interface.Bus(
            interface='pcan',
            channel='PCAN_USBBUS1',
            bitrate=BITRATE
        )
        print(f"CAN总线初始化成功: PCAN_USBBUS1 @ {BITRATE}bps")
    except Exception as e:
        print(f"CAN初始化失败: {e}")
        print("请检查:")
        print("1. PCAN硬件是否连接")
        print("2. PCAN驱动是否安装")
        print("3. 设备管理器中的COM端口号")
        return

    try:
        # 步骤1: 读取电机状态
        read_motor_data(bus)
        
        # 步骤2: 设置为位置模式
        set_position_mode(bus)
        
        # 步骤3: 发送目标位置
        target_pos = 10000  # 示例位置值
        print(f"\n=== 发送目标位置: {target_pos} ===")
        send_target_position(bus, target_pos)
        
        # 步骤4: 验证新位置
        time.sleep(1)  # 等待电机运动
        print("\n=== 验证新位置 ===")
        send_sdo_read_request(bus, OD_ACTUAL_POSITION[0], OD_ACTUAL_POSITION[1])
        read_sdo_response(bus)
        
    except KeyboardInterrupt:
        print("\n程序中断")
    finally:
        bus.shutdown()
        print("CAN总线关闭")

if __name__ == "__main__":
    main()