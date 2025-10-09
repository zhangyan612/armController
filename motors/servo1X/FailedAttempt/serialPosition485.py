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
        return None, f"CRC 校验失败: 收到 {received_crc.hex()}, 计算 {calculated_crc.hex()}"
    
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

# 发送 Modbus 请求并返回解析结果 (485优化版)
def send_modbus_request(
    ser: serial.Serial,
    request: bytes,
    max_retries: int = 3,
    delay_before_read: float = 0.1,  # 485通信需要等待时间
    timeout: float = 0.5             # 485通信需要更长超时
) -> Tuple[Optional[List[int]], str]:
    for attempt in range(max_retries):
        try:
            # 清空输入缓冲区
            ser.reset_input_buffer()
            
            # 发送请求
            ser.write(request)
            
            # 485通信需要等待方向切换
            time.sleep(delay_before_read)
            
            # 读取响应
            response = ser.read(1024)
            
            if not response:
                print(f"⚠️ 尝试 {attempt+1}/{max_retries}: 未收到响应")
                continue
                
            # 解析响应
            registers, error = parse_modbus_response(response)
            if error:
                print(f"⚠️ 尝试 {attempt+1}/{max_retries}: 响应解析错误 - {error}")
                continue
                
            return registers, ""
            
        except Exception as e:
            print(f"⚠️ 尝试 {attempt+1}/{max_retries}: 通信异常 - {str(e)}")
    
    return None, f"所有 {max_retries} 次尝试均失败"

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
    registers, error = send_modbus_request(ser, request)
    
    if error:
        print(f"❌ 清错失败: {error}")
        return False
    
    # 验证响应
    if registers and len(registers) >= 2:
        if registers[0] == 62 and registers[1] == 0x86:
            print("✅ 错误已清除")
            return True
    
    print("❌ 清错响应验证失败")
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
    else:
        data["电流"] = float('nan')

    # 总线电压（地址31）
    request = build_modbus_request(slave_id, 0x03, 31, 1)
    registers, error = send_modbus_request(ser, request)
    if registers and len(registers) == 1:
        data["电压"] = registers[0] * 0.1
    else:
        data["电压"] = float('nan')

    # 实际速度（地址42、43）
    request = build_modbus_request(slave_id, 0x03, 42, 2)
    registers, error = send_modbus_request(ser, request)
    if registers and len(registers) == 2:
        combined = (registers[0] << 16) | registers[1]
        if combined >= 0x80000000:  # 负数
            combined -= 0x100000000
        data["速度"] = combined * 0.001
    else:
        data["速度"] = float('nan')

    # 实际位置（地址44、45）
    request = build_modbus_request(slave_id, 0x03, 44, 2)
    registers, error = send_modbus_request(ser, request)
    if registers and len(registers) == 2:
        combined = (registers[0] << 16) | registers[1]
        if combined >= 0x80000000:  # 负数
            combined -= 0x100000000
        data["位置"] = combined
    else:
        data["位置"] = float('nan')

    # 设备温度（地址30）
    request = build_modbus_request(slave_id, 0x03, 30, 1)
    registers, error = send_modbus_request(ser, request)
    if registers and len(registers) == 1:
        data["温度"] = registers[0] * 0.1
    else:
        data["温度"] = float('nan')

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
def wait_for_position(ser: serial.Serial, slave_id: int, target_position: int, tolerance: int = 10, timeout: float = 15.0) -> bool:
    """等待电机到达目标位置"""
    start_time = time.time()
    last_position = None
    print(f"⏳ 等待位置到达 {target_position} ±{tolerance}...")
    
    while time.time() - start_time < timeout:
        # 读取实际位置
        data = read_realtime_data(ser, slave_id)
        current_position = data.get("位置")
        
        if current_position is None or current_position is float('nan'):
            print("❌ 无法读取位置数据")
            time.sleep(0.5)
            continue
        
        # 显示位置变化
        if last_position is not None and last_position != current_position:
            diff = current_position - last_position
            direction = "正转" if diff > 0 else "反转"
            print(f"位置变化: {last_position} → {current_position} ({direction}, 差值: {abs(diff)})")
        last_position = current_position
        
        # 检查是否到达目标位置
        if abs(current_position - target_position) <= tolerance:
            print(f"✅ 已到达目标位置 {target_position} (实际位置: {current_position})")
            return True
        
        # 打印当前位置
        print(f"当前位置: {current_position}, 目标: {target_position}, 差值: {abs(current_position - target_position)}")
        time.sleep(0.3)  # 避免过度查询
    
    print(f"⏰ 等待位置超时 (目标: {target_position}, 当前: {current_position})")
    return False

# 485接口配置函数
def configure_serial_for_485(ser: serial.Serial) -> None:
    """配置串口对象以优化485通信"""
    # 485通信通常需要更长的超时时间
    ser.timeout = 0.5
    ser.write_timeout = 0.5
    
    # 尝试配置硬件流控制（部分485转换器需要）
    try:
        ser.rtscts = False
        ser.dsrdtr = False
        # 部分485转换器使用RTS控制方向
        ser.rts = False  # 初始为接收模式
    except:
        print("⚠️ 该平台不支持RTS控制，485方向切换可能由硬件自动处理")

# 位置控制主函数（485优化版）
def position_control_demo_485(
    port: str = "COM3",  # 485接口通常使用不同的COM端口
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
    
    # 配置485优化参数
    configure_serial_for_485(ser)

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
            if isinstance(v, float) and v != v:  # 检查NaN
                print(f" - {k}: 读取失败")
            else:
                print(f" - {k}: {v:.3f}" if k in ["电流", "速度", "电压", "温度"] else f" - {k}: {v}")
        
        # 3. 设置工作模式为位置模式 (7)
        if not set_work_mode(ser, slave_id, 7):
            print("❌ 设置工作模式失败，无法继续")
            return
        
        # 4. 使能电机 (控制字 0xF = 15)
        if not set_control_word(ser, slave_id, 0xF):
            print("❌ 设置控制字失败，无法继续")
            return
        
        # 5. 设置目标位置为0
        if not set_target_position(ser, slave_id, 0):
            print("❌ 设置目标位置失败，无法继续")
            return
        
        # 6. 等待到达位置0
        if not wait_for_position(ser, slave_id, 0, tolerance=10, timeout=20.0):
            print("❌ 未能到达位置0")
            return
        
        # 7. 设置目标位置为6000
        if not set_target_position(ser, slave_id, 6000):
            print("❌ 设置目标位置失败，无法继续")
            return
        
        # 8. 等待到达位置6000
        if not wait_for_position(ser, slave_id, 6000, tolerance=10, timeout=20.0):
            print("❌ 未能到达位置6000")
            return
        
        # 9. 读取最终位置
        final_data = read_realtime_data(ser, slave_id)
        print("📊 最终实时数据：")
        for k, v in final_data.items():
            if isinstance(v, float) and v != v:  # 检查NaN
                print(f" - {k}: 读取失败")
            else:
                print(f" - {k}: {v:.3f}" if k in ["电流", "速度", "电压", "温度"] else f" - {k}: {v}")
        
        print("🎯 位置控制完成")

    except Exception as e:
        print(f"❌ 异常发生: {str(e)}")
    finally:
        # 松轴（安全停止）
        try:
            set_control_word(ser, slave_id, 0x6)  # 松轴命令
            print("🔓 发送松轴命令")
        except:
            pass
        
        ser.close()
        print("🔌 串口已关闭")

# 示例调用
if __name__ == "__main__":
    # 485接口位置控制演示
    print("🚀 开始485接口位置控制演示")
    position_control_demo_485(
        port="COM4",  # 修改为实际的485接口COM端口
        slave_id=1,
        baudrate=38400
    )