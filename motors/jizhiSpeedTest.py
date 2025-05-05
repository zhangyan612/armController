import can
import time
import struct

class MotorController:
    def __init__(self, interface='pcan', can_channel='PCAN_USBBUS1', bitrate=1000000):
        self.bus = can.interface.Bus(
            interface=interface,
            channel=can_channel,
            bitrate=bitrate
        )
        
        # 电机参数配置（需根据实际型号设置）
        self.Pmax = 1.0        # 位置满量程（圈数）
        self.Vmax = 200.0      # 速度满量程（rad/s）
        self.Imax = 4.0        # 电流满量程（A）
        
        # 控制指令集
        self.commands = {
            'mechanical_zero': bytes.fromhex('FF FF FF FF FF FF FF FE'),
            'start_motor':     bytes.fromhex('FF FF FF FF FF FF FF FC'),
            'stop_motor':      bytes.fromhex('FF FF FF FF FF FF FF FD'),
            'torque_mode':     bytes.fromhex('FF FF FF FF FF FF FF F9'),
            'velocity_mode':   bytes.fromhex('FF FF FF FF FF FF FF FA'),
            'position_mode':   bytes.fromhex('FF FF FF FF FF FF FF FB')
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

    def _value_to_position_code(self, degrees):
        """将角度转换为位置码值"""
        position_ratio = degrees / (self.Pmax * 360)
        code = int(position_ratio * 0x8000 + 0x8000)
        return max(0, min(0xFFFF, code))

    def _value_to_speed_code(self, rad_s):
        """将速度转换为12位码值"""
        speed_ratio = rad_s / self.Vmax
        code = int(speed_ratio * 0x800 + 0x800)
        return max(0, min(0xFFF, code))

    def _value_to_current_code(self, ampere):
        """将电流转换为12位码值"""
        current_ratio = ampere / self.Imax
        code = int(current_ratio * 0x800 + 0x800)
        return max(0, min(0xFFF, code))

    def send_position_command(self, degrees, max_speed, max_current, Kp, Kd):
        """发送位置控制命令（三环模式）"""
        pos_code = self._value_to_position_code(degrees)
        vel_code = self._value_to_speed_code(max_speed)
        cur_code = self._value_to_current_code(max_current)
        
        # 构造数据包（示例：C0 00 C0 01 00 02 0C 00）
        data = [
            (pos_code >> 8) & 0xFF,
            pos_code & 0xFF,
            (vel_code >> 4) & 0xFF,
            ((vel_code & 0x0F) << 4) | ((Kp >> 8) & 0x0F),
            Kp & 0xFF,
            (Kd >> 4) & 0xFF,
            ((Kd & 0x0F) << 4) | ((cur_code >> 8) & 0x0F),
            cur_code & 0xFF
        ]
        self._send_raw(bytes(data))

    def send_speed_command(self, target_speed, max_current):
        """发送速度控制命令"""
        vel_code = self._value_to_speed_code(target_speed)
        cur_code = self._value_to_current_code(max_current)
        
        data = [
            (vel_code >> 4) & 0xFF,
            ((vel_code & 0x0F) << 4) | ((cur_code >> 8) & 0x0F),
            cur_code & 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00
        ]
        self._send_raw(bytes(data[:8]))

    def send_mechanical_zero(self):
        """发送机械零位命令"""
        if self.bus.state != can.bus.BusState.PASSIVE:
            self._send_raw(self.commands['mechanical_zero'])
            print("机械零位已设置（需在未使能状态下执行）")
        else:
            print("错误：电机仍在使能状态")

    def _send_raw(self, data, can_id=0x1):
        """通用数据发送方法"""
        message = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=False
        )
        try:
            self.bus.send(message)
            print(f"发送成功: {data.hex().upper()}")
        except can.CanError as e:
            print(f"发送失败: {e}")

    def parse_response(self, data):
        """解析电机响应数据"""
        if len(data) != 8:
            return {"error": "无效数据长度"}
            
        # 解析位置（16位）
        pos_code = (data[0] << 8) | data[1]
        position = ((pos_code - 0x8000) / 0x8000) * 360 * self.Pmax
        
        # 解析速度（12位）
        vel_high = data[2]
        vel_low = (data[3] & 0xF0) >> 4
        vel_code = (vel_high << 4) | vel_low
        velocity = ((vel_code - 0x800) / 0x800) * self.Vmax
        
        # 解析电流（12位）
        cur_high = data[3] & 0x0F
        cur_low = data[4]
        cur_code = (cur_high << 8) | cur_low
        current = ((cur_code - 0x800) / 0x800) * self.Imax
        
        return {
            "position_deg": round(position, 2),
            "velocity_rads": round(velocity, 3),
            "current_a": round(current, 4),
            "raw_data": data.hex().upper()
        }

    def monitor(self, timeout=10):
        """监听电机反馈"""
        start_time = time.time()
        print(f"启动监听（超时：{timeout}s）...")
        while time.time() - start_time < timeout:
            msg = self.bus.recv(timeout=0.5)
            if msg:
                parsed = self.parse_response(msg.data)
                print(f"[ID 0x{msg.arbitration_id:X}] {parsed}")

# 测试用例
if __name__ == "__main__":
    mc = MotorController()
    
    # 测试机械零位
    mc.send_mechanical_zero()
    
    mc.send_command('start_motor')
    mc.monitor(timeout=3)

    # 测试位置命令（180度，100rad/s，2A，Kp=0x100，Kd=0x020）
    print("\n发送位置命令：")
    mc.send_position_command(
        degrees=180,
        max_speed=100,
        max_current=2,
        Kp=0x100,
        Kd=0x020
    )
    mc.monitor(timeout=2)
    
    # 测试速度命令（-50rad/s，1A）
    print("\n发送速度命令：")
    mc.send_speed_command(
        target_speed=-50,
        max_current=1
    )
    mc.monitor(timeout=2)


    mc.send_command('stop_motor')
    mc.monitor(timeout=5)
