import can
import struct
from typing import List, Tuple, Optional, Dict, Any
import time

# CRC16 校验函数
def calculate_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack("<H", crc)

# 构造 Modbus RTU 请求报文
def build_modbus_request(
    slave_id: int,
    function_code: int,
    start_address: int,
    num_registers: int,
) -> bytes:
    request = bytearray([
        slave_id,
        function_code,
        (start_address >> 8) & 0xFF,
        start_address & 0xFF,
        (num_registers >> 8) & 0xFF,
        num_registers & 0xFF
    ])
    request += calculate_crc16(request)
    return bytes(request)

# 写单个寄存器
def build_modbus_request_write_single(
    slave_id: int,
    register_address: int,
    value: int
) -> bytes:
    request = bytearray([
        slave_id,
        0x06,
        (register_address >> 8) & 0xFF,
        register_address & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF
    ])
    request += calculate_crc16(request)
    return bytes(request)

# 解析 Modbus RTU 响应报文
def parse_modbus_response(response: bytes) -> Tuple[Optional[List[int]], str]:
    if len(response) < 5:
        return None, "响应数据太短"
    
    slave_id = response[0]
    function_code = response[1]
    byte_count = response[2]

    if len(response) < 3 + byte_count + 2:
        return None, "响应数据不完整"

    data_bytes = response[3:-2]
    if len(data_bytes) != byte_count:
        return None, f"数据字节数不匹配: {len(data_bytes)} vs {byte_count}"

    registers = []
    for i in range(0, len(data_bytes), 2):
        high = data_bytes[i]
        low = data_bytes[i+1]
        registers.append((high << 8) | low)

    received_crc = response[-2:]
    calculated_crc = calculate_crc16(response[:-2])
    if received_crc != calculated_crc:
        return None, "CRC 校验失败"

    return registers, ""

# 通过 CAN 发送 Modbus 请求并接收响应
def send_can_modbus_request(
    bus: can.BusABC,
    slave_id: int,
    request_data: bytes
) -> Tuple[Optional[List[int]], str]:
    # 假设仲裁ID为 slave_id（标准 11-bit ID）
    arbitration_id = slave_id
    msg = can.Message(arbitration_id=arbitration_id, data=request_data, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError:
        return None, "CAN 发送失败"

    # 等待响应，假设响应 ID 为 slave_id + 0x100
    response_id = slave_id + 0x100
    timeout = 2.0
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            response_msg = bus.recv(timeout=0.1)
            if response_msg and response_msg.arbitration_id == response_id:
                response_data = response_msg.data
                registers, error = parse_modbus_response(response_data)
                return registers, error
        except can.CanError:
            continue
    return None, "未收到响应"

# 读取错误状态并解析
def read_error_status(bus: can.BusABC, slave_id: int) -> Tuple[Optional[List[int]], str, str]:
    request = build_modbus_request(slave_id, 0x03, 50, 2)
    registers, error = send_can_modbus_request(bus, slave_id, request)
    
    if error:
        return None, error, ""
    
    if not registers or len(registers) < 2:
        return None, "寄存器数据不完整", ""
    
    # 解析错误状态（按协议文档）
    err_code1 = registers[0]
    err_code2 = registers[1]
    error_messages = []
    
    # 解析错误码1
    if err_code1 & 0b10:
        error_messages.append("电流传感器错误")
    if err_code1 & 0b1000:
        error_messages.append("编码器错误")
    if err_code1 & 0b10000:
        error_messages.append("霍尔信号错误")
    if err_code1 & 0b100000:
        error_messages.append("励磁错误")
    if err_code1 & 0b1000000:
        error_messages.append("跟随超差错误")
    if err_code1 & 0b10000000:
        error_messages.append("低压错误")
    
    # 解析错误码2
    if err_code2 & 0b00000001:
        error_messages.append("高压错误")
    if err_code2 & 0b00000010:
        error_messages.append("过温错误")
    if err_code2 & 0b00000100:
        error_messages.append("过流错误")
    if err_code2 & 0b00001000:
        error_messages.append("过载错误")
    
    error_str = ", ".join(error_messages) if error_messages else "无错误"
    
    return registers, "", error_str

# 清除错误（控制字地址62 = 0x86）
def clear_error(bus: can.BusABC, slave_id: int) -> bool:
    request = build_modbus_request_write_single(slave_id, 62, 0x86)
    print(f"📤 发送清错命令: {request.hex()}")
    registers, error = send_can_modbus_request(bus, slave_id, request)
    if error:
        print(f"❌ 清错失败: {error}")
        return False
    if registers and registers[0] == 0x86:
        print("✅ 错误已清除")
        return True
    else:
        print("❌ 清错失败")
        return False

# 读取实时数据
def read_realtime_data(bus: can.BusABC, slave_id: int) -> Dict[str, float]:
    data = {}

    # 实时电流（地址40、41）
    request = build_modbus_request(slave_id, 0x03, 40, 2)
    registers, error = send_can_modbus_request(bus, slave_id, request)
    if registers and len(registers) == 2:
        combined = (registers[0] << 16) | registers[1]
        if combined >= 0x80000000:  # 负数
            combined -= 0x100000000
        data["电流"] = combined * 0.001

    # 总线电压（地址31）
    request = build_modbus_request(slave_id, 0x03, 31, 1)
    registers, error = send_can_modbus_request(bus, slave_id, request)
    if registers and len(registers) == 1:
        data["电压"] = registers[0] * 0.1

    # 实际速度（地址42、43）
    request = build_modbus_request(slave_id, 0x03, 42, 2)
    registers, error = send_can_modbus_request(bus, slave_id, request)
    if registers and len(registers) == 2:
        combined = (registers[0] << 16) | registers[1]
        if combined >= 0x80000000:  # 负数
            combined -= 0x100000000
        data["速度"] = combined * 0.001

    # 实际位置（地址44、45）
    request = build_modbus_request(slave_id, 0x03, 44, 2)
    registers, error = send_can_modbus_request(bus, slave_id, request)
    if registers and len(registers) == 2:
        combined = (registers[0] << 16) | registers[1]
        if combined >= 0x80000000:  # 负数
            combined -= 0x100000000
        data["位置"] = combined

    # 设备温度（地址30）
    request = build_modbus_request(slave_id, 0x03, 30, 1)
    registers, error = send_can_modbus_request(bus, slave_id, request)
    if registers and len(registers) == 1:
        data["温度"] = registers[0] * 0.1

    return data

# 主函数：读取错误状态并获取实时数据
def read_and_clear_errors_and_get_data_can(
    channel: str = 'PCAN_USBBUS1',
    bitrate: int = 500000,
    slave_id: int = 1
) -> None:
    try:
        bus = can.interface.Bus(channel=channel, interface='pcan', bitrate=bitrate)
        print("✅ CAN 总线已打开")
    except Exception as e:
        print(f"❌ CAN 初始化失败: {e}")
        return

    try:
        # 读取错误状态
        registers, error, error_messages = read_error_status(bus, slave_id)
        if error:
            print(f"❌ 错误状态读取失败: {error}")
        else:
            if registers:
                print(f"🔍 错误状态（地址50、51）: {hex(registers[0])}, {hex(registers[1])}")
                print(f"⚠️ 错误详情: {error_messages}")

                if registers[0] != 0 or registers[1] != 0:
                    print("⚠️ 检测到错误，正在清除...")
                    clear_error(bus, slave_id)
                else:
                    print("✅ 无错误")

        # 读取实时数据
        realtime_data = read_realtime_data(bus, slave_id)
        print("📊 实时数据：")
        for k, v in realtime_data.items():
            print(f" - {k}: {v:.3f}" if k in ["电流", "速度", "电压", "温度"] else f" - {k}: {v}")

    except Exception as e:
        print(f"❌ 异常发生: {str(e)}")
    finally:
        bus.shutdown()
        print("🔌 CAN 总线已关闭")

# 示例调用
if __name__ == "__main__":
    # not working 
    read_and_clear_errors_and_get_data_can(slave_id=1)