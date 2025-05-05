import can
import time
import struct

class MotorController:
    def __init__(self, interface='pcan', can_channel='PCAN_USBBUS1', bitrate=1000000):
        self.bus = can.interface.Bus(
            interface=interface,
            channel=can_channel,
            bitrate=bitrate,
            receive_own_messages=True
        )
        
        # 控制指令集（新增）
        self.commands = {
            'start_motor':    bytes.fromhex('FF FF FF FF FF FF FF FC'),
            'stop_motor':    bytes.fromhex('FF FF FF FF FF FF FF FD'),
            'mechanical_zero':bytes.fromhex('FF FF FF FF FF FF FF FE'),
            'torque_mode':    bytes.fromhex('FF FF FF FF FF FF FF F9'),
            'velocity_mode':  bytes.fromhex('FF FF FF FF FF FF FF FA'),
            'position_mode': bytes.fromhex('FF FF FF FF FF FF FF FB')
        }

    def send_command(self, command_name, can_id=0x1):
        """发送预定义控制指令"""
        if command_name not in self.commands:
            raise ValueError(f"未知指令: {command_name}")
        
        message = can.Message(
            arbitration_id=can_id,
            data=self.commands[command_name],
            is_extended_id=False
        )
        
        try:
            self.bus.send(message)
            print(f"指令发送成功: {command_name}")
        except can.CanError as e:
            print(f"发送失败: {e}")

    def _parse_response(self, data):
        """增强型数据解析方法"""
        if len(data) == 8:  # 解析8字节响应
            return self._parse_8byte_data(data)
        elif len(data) == 6:  # 解析6字节响应
            return self._parse_6byte_data(data)
        else:
            return {"error": f"未知数据格式: {data.hex().upper()}"}

    def _parse_8byte_data(self, data):
        """解析8字节反馈数据（示例：FFA495D3E83E8866）"""
        try:
            # 将字节数据转换为两个32位整数（大端序）
            val1, val2 = struct.unpack('>II', data)
            
            # 位置解析（示例FFA495D3）
            position_raw = val1 >> 16
            position_deg = self._twos_complement(position_raw, 16) * 0.01
            
            # 速度解析（示例E83E）
            velocity_raw = (val2 >> 16) & 0xFFFF
            velocity_rads = self._twos_complement(velocity_raw, 16) * 0.001
            
            # 电流解析（示例8866）
            current_raw = val2 & 0xFFFF
            current_a = self._twos_complement(current_raw, 16) * 0.0001
            
            return {
                'position': f"{position_deg:.2f}°",
                'velocity': f"{velocity_rads:.3f} rad/s",
                'current': f"{current_a:.4f} A"
            }
        except Exception as e:
            return {"error": f"8字节解析失败: {str(e)}"}

    def _parse_6byte_data(self, data):
        """解析6字节反馈数据（示例：8000ABA000000A00）"""
        try:
            # 组合成64位数据（补充两个字节）
            extended_data = data + b'\x00\x00'
            value = struct.unpack('>Q', extended_data)[0]
            
            # 位置解析（20位有符号）
            position_raw = (value >> 44) & 0xFFFFF
            position_deg = self._twos_complement(position_raw, 20) * 0.1
            
            # 速度解析（20位有符号）
            velocity_raw = (value >> 24) & 0xFFFFF
            velocity_rads = self._twos_complement(velocity_raw, 20) * 0.01
            
            # 电流解析（24位有符号）
            current_raw = value & 0xFFFFFF
            current_a = self._twos_complement(current_raw, 24) * 0.001
            
            return {
                'position': f"{position_deg:.1f}°",
                'velocity': f"{velocity_rads:.2f} rad/s",
                'current': f"{current_a:.3f} A"
            }
        except Exception as e:
            return {"error": f"6字节解析失败: {str(e)}"}

    @staticmethod
    def _twos_complement(value, bits):
        """二进制补码转换"""
        if (value & (1 << (bits - 1))) != 0:
            value = value - (1 << bits)
        return value

    def monitor(self, timeout=10):
        """增强型监控循环"""
        start_time = time.time()
        print(f"启动监控，超时时间：{timeout}秒")
        while time.time() - start_time < timeout:
            msg = self.bus.recv(timeout=0.5)
            if msg:
                parsed = self._parse_response(msg.data)
                print(f"ID:0x{msg.arbitration_id:X} | 数据:{msg.data.hex().upper()} | 解析:{parsed}")
            else:
                print("等待数据中...")

# 测试用例
if __name__ == "__main__":
    mc = MotorController()
    
    # 测试不同模式
    modes = [
        ('start_motor', 5),
        ('position_mode', 3),
        ('velocity_mode', 3),
        ('torque_mode', 3),
        ('stop_motor', 5)
    ]
    
    for command, duration in modes:
        print(f"\n执行命令: {command}")
        mc.send_command(command)
        mc.monitor(timeout=duration)
        time.sleep(1)