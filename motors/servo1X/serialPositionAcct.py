import serial
import struct
import time
from typing import List, Tuple, Optional, Dict

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

# 解析 Modbus RTU 响应报文
def parse_modbus_response(response: bytes) -> Tuple[Optional[List[int]], str]:
    if len(response) < 5:
        return None, "响应数据太短"
    
    slave_id = response[0]
    function_code = response[1]
    
    # 处理不同功能码的响应
    if function_code == 0x03:  # 读多个寄存器
        byte_count = response[2]
        if len(response) < 3 + byte_count + 2:
            return None, "响应数据不完整"
        data_bytes = response[3:3+byte_count]
        if len(data_bytes) != byte_count:
            return None, f"数据字节数不匹配: {len(data_bytes)} vs {byte_count}"
        
        registers = []
        for i in range(0, len(data_bytes), 2):
            high = data_bytes[i]
            low = data_bytes[i+1]
            registers.append((high << 8) | low)
    
    elif function_code == 0x06:  # 写单个寄存器
        if len(response) != 8:
            return None, "响应长度错误"
        registers = [
            (response[2] << 8) | response[3],  # 地址
            (response[4] << 8) | response[5]   # 值
        ]
    
    elif function_code == 0x10:  # 写多个寄存器
        if len(response) != 8:
            return None, "响应长度错误"
        registers = [
            (response[2] << 8) | response[3],  # 起始地址
            (response[4] << 8) | response[5]   # 寄存器数量
        ]
    
    else:
        return None, f"不支持的功能码: {function_code}"
    
    # CRC 校验
    received_crc = response[-2:]
    calculated_crc = calculate_crc16(response[:-2])
    if received_crc != calculated_crc:
        return None, "CRC 校验失败"
    
    return registers, ""

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

# 写多个寄存器
def build_modbus_request_write_multiple(
    slave_id: int,
    start_address: int,
    values: List[int]
) -> bytes:
    num_registers = len(values)
    byte_count = num_registers * 2
    
    request = bytearray([
        slave_id,
        0x10,
        (start_address >> 8) & 0xFF,
        start_address & 0xFF,
        (num_registers >> 8) & 0xFF,
        num_registers & 0xFF,
        byte_count
    ])
    
    for value in values:
        request.append((value >> 8) & 0xFF)
        request.append(value & 0xFF)
    
    request += calculate_crc16(request)
    return bytes(request)

# 发送 Modbus 请求并返回解析结果
def send_modbus_request(
    ser: serial.Serial,
    request: bytes,
    expected_byte_count: int = -1
) -> Tuple[Optional[List[int]], str]:
    ser.write(request)
    response = ser.read(1024)
    if not response:
        return None, "未收到响应"

    registers, error = parse_modbus_response(response)
    if error:
        return None, error

    return registers, ""

# 读取错误状态并解析
def read_error_status(ser: serial.Serial, slave_id: int) -> Tuple[Optional[List[int]], str, str]:
    request = build_modbus_request(slave_id, 0x03, 50, 2)
    registers, error = send_modbus_request(ser, request)
    
    if error:
        return None, error, ""
    
    if not registers or len(registers) < 2:
        return None, "寄存器数据不完整", ""
    
    # 解析错误状态
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

# 清除错误
def clear_error(ser: serial.Serial, slave_id: int) -> bool:
    request = build_modbus_request_write_single(slave_id, 62, 0x86)
    print(f"📤 发送清错命令: {request.hex()}")
    ser.write(request)
    response = ser.read(1024)
    if response == request:
        print("✅ 错误已清除")
        return True
    else:
        print("❌ 清错失败")
        return False

# 读取实时数据
def read_realtime_data(ser: serial.Serial, slave_id: int) -> Dict[str, float]:
    data = {}

    # 实时电流（地址40、41）
    request = build_modbus_request(slave_id, 0x03, 40, 2)
    registers, error = send_modbus_request(ser, request)
    if registers and len(registers) == 2:
        # 32位有符号整数（高位在前）
        combined = (registers[0] << 16) | registers[1]
        if combined >= 0x80000000:  # 负数
            combined -= 0x100000000
        data["电流"] = combined * 0.001

    # 总线电压（地址31）
    request = build_modbus_request(slave_id, 0x03, 31, 1)
    registers, error = send_modbus_request(ser, request)
    if registers and len(registers) == 1:
        data["电压"] = registers[0] * 0.1

    # 实际速度（地址42、43）
    request = build_modbus_request(slave_id, 0x03, 42, 2)
    registers, error = send_modbus_request(ser, request)
    if registers and len(registers) == 2:
        combined = (registers[0] << 16) | registers[1]
        if combined >= 0x80000000:  # 负数
            combined -= 0x100000000
        data["速度"] = combined * 0.001

    # 实际位置（地址44、45）
    request = build_modbus_request(slave_id, 0x03, 44, 2)
    registers, error = send_modbus_request(ser, request)
    if registers and len(registers) == 2:
        combined = (registers[0] << 16) | registers[1]
        if combined >= 0x80000000:  # 负数
            combined -= 0x100000000
        data["位置"] = combined

    # 设备温度（地址30）
    request = build_modbus_request(slave_id, 0x03, 30, 1)
    registers, error = send_modbus_request(ser, request)
    if registers and len(registers) == 1:
        data["温度"] = registers[0] * 0.1

    return data

# 设置工作模式
def set_work_mode(ser: serial.Serial, slave_id: int, mode: int) -> bool:
    """设置工作模式 (地址60)"""
    request = build_modbus_request_write_single(slave_id, 60, mode)
    print(f"📤 设置工作模式为 {mode}: {request.hex()}")
    registers, error = send_modbus_request(ser, request)
    
    if error:
        print(f"❌ 设置工作模式失败: {error}")
        return False
    
    # 验证设置是否成功
    if registers and len(registers) >= 2:
        if registers[0] == 60 and registers[1] == mode:
            print(f"✅ 工作模式已设置为 {mode}")
            return True
    
    print("❌ 工作模式设置验证失败")
    return False

# 设置控制字
def set_control_word(ser: serial.Serial, slave_id: int, value: int) -> bool:
    """设置控制字 (地址62)"""
    request = build_modbus_request_write_single(slave_id, 62, value)
    print(f"📤 设置控制字为 {value} (0x{value:X}): {request.hex()}")
    registers, error = send_modbus_request(ser, request)
    
    if error:
        print(f"❌ 设置控制字失败: {error}")
        return False
    
    # 验证设置是否成功
    if registers and len(registers) >= 2:
        if registers[0] == 62 and registers[1] == value:
            print(f"✅ 控制字已设置为 0x{value:X}")
            return True
    
    print("❌ 控制字设置验证失败")
    return False

# 设置目标位置
def set_target_position(ser: serial.Serial, slave_id: int, position: int) -> bool:
    """设置目标位置 (地址68、69)"""
    # 将32位位置拆分为两个16位寄存器值
    high_word = (position >> 16) & 0xFFFF
    low_word = position & 0xFFFF
    values = [high_word, low_word]
    
    request = build_modbus_request_write_multiple(slave_id, 68, values)
    print(f"📤 设置目标位置为 {position}: {request.hex()}")
    registers, error = send_modbus_request(ser, request)
    
    if error:
        print(f"❌ 设置目标位置失败: {error}")
        return False
    
    # 验证设置是否成功
    if registers and len(registers) >= 2:
        if registers[0] == 68 and registers[1] == 2:  # 起始地址68，寄存器数量2
            print(f"✅ 目标位置已设置为 {position}")
            return True
    
    print("❌ 目标位置设置验证失败")
    return False

# 等待位置到达
def wait_for_position(ser: serial.Serial, slave_id: int, target_position: int, tolerance: int = 10, timeout: float = 10.0) -> bool:
    """等待电机到达目标位置"""
    start_time = time.time()
    print(f"⏳ 等待位置到达 {target_position} ±{tolerance}...")
    
    while time.time() - start_time < timeout:
        # 读取实际位置
        data = read_realtime_data(ser, slave_id)
        current_position = data.get("位置")
        
        if current_position is None:
            print("❌ 无法读取位置数据")
            return False
        
        # 检查是否到达目标位置
        if abs(current_position - target_position) <= tolerance:
            print(f"✅ 已到达目标位置 {target_position} (实际位置: {current_position})")
            return True
        
        # 打印当前位置
        print(f"当前位置: {current_position}, 目标: {target_position}, 差值: {abs(current_position - target_position)}")
        time.sleep(0.2)  # 避免过度查询
    
    print(f"⏰ 等待位置超时 (目标: {target_position}, 当前: {current_position})")
    return False

# 设置32位参数（用于梯形速度/加速度/减速度）
def set_32bit_parameter(
    ser: serial.Serial,
    slave_id: int,
    start_address: int,
    value: int
) -> bool:
    """设置32位参数（需要写入两个连续的寄存器）"""
    # 将32位值拆分为两个16位寄存器值
    high_word = (value >> 16) & 0xFFFF
    low_word = value & 0xFFFF
    values = [high_word, low_word]
    
    request = build_modbus_request_write_multiple(slave_id, start_address, values)
    print(f"📤 设置地址 {start_address} 为 {value} (高位:0x{high_word:X}, 低位:0x{low_word:X}): {request.hex()}")
    registers, error = send_modbus_request(ser, request)
    
    if error:
        print(f"❌ 设置32位参数失败: {error}")
        return False
    
    # 验证设置是否成功
    if registers and len(registers) >= 2:
        if registers[0] == start_address and registers[1] == 2:  # 起始地址和寄存器数量
            print(f"✅ 32位参数设置成功 (地址:{start_address}, 值:{value})")
            return True
    
    print("❌ 32位参数设置验证失败")
    return False

# 位置控制主函数（添加轨迹参数设置）
def position_control_demo(
    port: str = "COM22",
    baudrate: int = 38400,
    slave_id: int = 1,
    timeout: float = 2
) -> None:
    ser = serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout
    )
    if not ser.is_open:
        print("❌ 串口未打开")
        return

    print("✅ 串口已打开")

    try:
        # 1. 读取并清除错误
        registers, error, error_messages = read_error_status(ser, slave_id)
        if error:
            print(f"❌ 错误状态读取失败: {error}")
        else:
            if registers:
                print(f"🔍 错误状态（地址50、51）: {hex(registers[0])}, {hex(registers[1])}")
                print(f"⚠️ 错误详情: {error_messages}")

                if registers[0] != 0 or registers[1] != 0:
                    print("⚠️ 检测到错误，正在清除...")
                    if not clear_error(ser, slave_id):
                        print("❌ 清除错误失败，无法继续")
                        return
                else:
                    print("✅ 无错误")
        
        # 2. 读取实时数据
        realtime_data = read_realtime_data(ser, slave_id)
        print("📊 初始实时数据：")
        for k, v in realtime_data.items():
            print(f" - {k}: {v:.3f}" if k in ["电流", "速度", "电压", "温度"] else f" - {k}: {v}")
        
        # 3. 设置轨迹参数 (梯形速度/加速度/减速度)
        trapezoidal_speed = 10000 
        trapezoidal_accel = 1000 
        trapezoidal_decel = 1000
        
        print(f"⚙️ 设置梯形速度: {trapezoidal_speed} (原始值:10000)")
        if not set_32bit_parameter(ser, slave_id, 222, trapezoidal_speed):
            print("❌ 设置梯形速度失败")
            return
            
        print(f"⚙️ 设置梯形加速度: {trapezoidal_accel} (原始值:1000)")
        if not set_32bit_parameter(ser, slave_id, 224, trapezoidal_accel):
            print("❌ 设置梯形加速度失败")
            return
            
        print(f"⚙️ 设置梯形减速度: {trapezoidal_decel} (原始值:1000)")
        if not set_32bit_parameter(ser, slave_id, 226, trapezoidal_decel):
            print("❌ 设置梯形减速度失败")
            return
        
        # 4. 设置工作模式为轨迹位置模式 (1)
        if not set_work_mode(ser, slave_id, 1):  # 模式1 = 轨迹位置模式
            print("❌ 设置工作模式失败，无法继续")
            return
        
        # 5. 使能电机 (控制字 0xF = 15)
        if not set_control_word(ser, slave_id, 0xF):
            print("❌ 设置控制字失败，无法继续")
            return
        
        # 6. 设置目标位置为0
        if not set_target_position(ser, slave_id, 0):
            print("❌ 设置目标位置失败，无法继续")
            return
        
        # 7. 等待到达位置0
        if not wait_for_position(ser, slave_id, 0, tolerance=50, timeout=15.0):
            print("❌ 未能到达位置0")
            # return
        
        # 8. 设置目标位置为10000
        if not set_target_position(ser, slave_id, 100000):
            print("❌ 设置目标位置失败，无法继续")
            return
        
        # 9. 等待到达位置10000
        if not wait_for_position(ser, slave_id, 100000, tolerance=50, timeout=15.0):
            print("❌ 未能到达位置10000")
            # return
        
        # 10. 读取最终位置
        final_data = read_realtime_data(ser, slave_id)
        print("📊 最终实时数据：")
        for k, v in final_data.items():
            print(f" - {k}: {v:.3f}" if k in ["电流", "速度", "电压", "温度"] else f" - {k}: {v}")
        
        print("🎯 轨迹位置控制完成")

    except Exception as e:
        print(f"❌ 异常发生: {str(e)}")
    finally:
        ser.close()
        print("🔌 串口已关闭")

# 示例调用
if __name__ == "__main__":
    position_control_demo(slave_id=1)