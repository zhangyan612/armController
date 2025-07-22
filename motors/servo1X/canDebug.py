import can
import time
import struct

# 配置参数
BITRATE = 500000
NODE_ID = 1  # 确保与STM32的RS485_Addr一致

# 使用STM32实际使用的ID格式
SDO_TX_ID = 0x600 + NODE_ID
SDO_RX_ID = 0x580 + NODE_ID
TPDO1_ID = 0x180 + NODE_ID
RPDO1_ID = 0x200 + NODE_ID

def debug_can_connection():
    print("=== CAN连接测试 ===")
    
    try:
        # 显式配置PCAN接口
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
        # 测试1: 发送SDO读取请求
        print("\n[测试1] 发送SDO读取请求 (0x1001 - 设备类型)")
        send_sdo_read_request(bus, 0x1001, 0x00)
        
        # 等待并打印所有响应
        wait_and_print_responses(bus, timeout=1.0)
        
        # 测试2: 发送RPDO
        print("\n[测试2] 发送RPDO控制命令")
        send_rpdo_test(bus)
        
        # 测试3: 监听TPDO
        print("\n[测试3] 监听TPDO (等待5秒)")
        listen_for_tpdo(bus, duration=5)
        
    except KeyboardInterrupt:
        print("\n测试中断")
    finally:
        bus.shutdown()
        print("CAN总线关闭")

def send_sdo_read_request(bus, index, subindex):
    """发送SDO读取请求 (兼容STM32格式)"""
    data = [
        0x40,           # SDO读取命令
        index & 0xFF,   # 索引低字节
        (index >> 8) & 0xFF,  # 索引高字节
        subindex,       # 子索引
        0x00, 0x00, 0x00, 0x00  # 填充
    ]
    msg = can.Message(
        arbitration_id=SDO_TX_ID,
        data=data,
        is_extended_id=False
    )
    bus.send(msg)
    print(f"发送SDO请求: ID=0x{SDO_TX_ID:X}, "
          f"Index=0x{index:04X}, Subindex=0x{subindex:02X}")

def send_rpdo_test(bus):
    """发送RPDO测试命令 (兼容STM32格式)"""
    # 使用大端序构造数据
    data = [
        0x00, 0x01,  # 工作模式 (位置模式)
        0x00, 0x0F,   # 控制字 (使能)
        0x00, 0x00, 0x00, 0x00  # 目标位置 (0)
    ]
    msg = can.Message(
        arbitration_id=RPDO1_ID,
        data=data,
        is_extended_id=False
    )
    bus.send(msg)
    print(f"发送RPDO: ID=0x{RPDO1_ID:X}, Data={[hex(x) for x in data]}")

def wait_and_print_responses(bus, timeout=1.0):
    """等待并打印所有CAN响应"""
    start_time = time.time()
    responses = []
    
    while time.time() - start_time < timeout:
        msg = bus.recv(timeout=0.1)
        if msg:
            responses.append(msg)
            print(f"收到消息: ID=0x{msg.arbitration_id:X}, "
                  f"DLC={msg.dlc}, Data={msg.data.hex()}")
    
    if not responses:
        print("未收到任何响应!")
        print("可能原因:")
        print("1. STM32未上电或未运行")
        print("2. CAN物理连接问题")
        print("3. 波特率不匹配 (当前500kbps)")
        print("4. 节点ID不匹配 (当前NODE_ID={})".format(NODE_ID))
    else:
        print(f"共收到 {len(responses)} 条消息")

def listen_for_tpdo(bus, duration=5):
    """监听TPDO消息"""
    print("监听TPDO... (等待电机周期性数据)")
    start_time = time.time()
    tpdo_count = 0
    
    while time.time() - start_time < duration:
        msg = bus.recv(timeout=0.1)
        if msg:
            # 检查是否是TPDO
            if msg.arbitration_id == TPDO1_ID:
                tpdo_count += 1
                # 尝试解析STM32格式的数据
                try:
                    # STM32使用大端序存储
                    raw_speed = msg.data[0] << 24 | msg.data[1] << 16 | msg.data[2] << 8 | msg.data[3]
                    raw_position = msg.data[4] << 24 | msg.data[5] << 16 | msg.data[6] << 8 | msg.data[7]
                    
                    # 转换为实际值 (根据协议)
                    speed_rpm = raw_speed / 1000.0  # 单位 0.001转/秒
                    position = raw_position  # 单位count
                    
                    print(f"TPDO: 速度={speed_rpm:.2f}RPM, 位置={position}")
                except:
                    print(f"TPDO原始数据: {msg.data.hex()}")
    
    if tpdo_count > 0:
        print(f"收到 {tpdo_count} 条TPDO消息")
    else:
        print("未收到TPDO消息! 可能原因:")
        print("1. 电机未配置为周期性发送TPDO")
        print("2. TPDO发送周期太长")
        print("3. TPDO ID不匹配 (当前预期0x{:X})".format(TPDO1_ID))

if __name__ == "__main__":
    # debug_can_connection()
    # 尝试不同的节点ID
    NODE_ID = 1
    # for node_id in range(254, 256):
    #     NODE_ID = node_id
    #     print(f"\n尝试节点ID: {node_id}")
    debug_can_connection()
