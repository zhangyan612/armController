import can
import time 

class MotorController:
    def __init__(self, interface='pcan', can_channel='PCAN_USBBUS1', bitrate=100000):
        # CAN总线初始化（Windows兼容配置）
        self.bus = can.interface.Bus(
            interface=interface,
            channel=can_channel,
            bitrate=bitrate
        )
        
    def _construct_tx_data(self, s_p_int, s_v_int, s_Kp_int, s_Kd_int, s_c_int):
        """构造发送数据包（位置模式）"""
        tx_data = [
            (s_p_int >> 8) & 0xFF,         # 位置高8位
            s_p_int & 0xFF,                # 位置低8位
            (s_v_int >> 4) & 0xFF,         # 速度高8位
            ((s_v_int & 0x0F) << 4) | ((s_Kp_int >> 8) & 0x0F),  # 速度低4位 + Kp高4位
            s_Kp_int & 0xFF,               # Kp低8位
            (s_Kd_int >> 4) & 0xFF,        # Kd高8位
            ((s_Kd_int & 0x0F) << 4) | ((s_c_int >> 8) & 0x0F),  # Kd低4位 + 力矩高4位
            s_c_int & 0xFF                 # 力矩低8位
        ]
        return bytes(tx_data)

    def send_position_command(self, can_id, pos, vel, Kp, Kd, current):
        """发送位置控制命令"""
        # 参数范围检查
        assert 0 <= pos <= 0xFFFF,     "位置参数超出16位范围"
        assert 0 <= vel <= 0xFFF,      "速度参数超出12位范围"
        assert 0 <= Kp <= 0xFFF,       "Kp参数超出12位范围"
        assert 0 <= Kd <= 0xFFF,       "Kd参数超出12位范围"
        assert 0 <= current <= 0xFFF,  "力矩参数超出12位范围"

        # 构造CAN数据帧
        message = can.Message(
            arbitration_id=can_id,
            data=self._construct_tx_data(pos, vel, Kp, Kd, current),
            is_extended_id=False
        )

        # 发送数据
        try:
            self.bus.send(message)
            print(f"成功发送位置指令: pos={pos}, vel={vel}, Kp={Kp}, Kd={Kd}, current={current}")
        except can.CanError as e:
            print(f"CAN发送失败: {e}")

    def parse_rx_data(self, rx_data):
        """解析接收数据"""
        if len(rx_data) < 6:
            raise ValueError("接收数据长度不足")

        # 位置解析（16位无符号）
        position = (rx_data[1] << 8) | rx_data[2]
        
        # 速度解析（12位有符号）
        vel_raw = ((rx_data[3] & 0xFF) << 4) | ((rx_data[4] & 0xF0) >> 4)
        velocity = vel_raw - 2048  # 转换为有符号
        
        # 力矩解析（12位有符号）
        current_raw = ((rx_data[4] & 0x0F) << 8) | rx_data[5]
        current = current_raw - 2048  # 转换为有符号

        return {
            'position': position,
            'velocity': velocity,
            'current': current
        }
    
    def send_raw_command(self, can_id, data_bytes):
        """发送原始数据指令（匹配上位机格式）"""
        message = can.Message(
            arbitration_id=can_id,
            data=data_bytes,
            is_extended_id=False
        )
        try:
            self.bus.send(message)
            print(f"成功发送原始指令: ID=0x{can_id:X}, Data={data_bytes.hex().upper()}")
        except can.CanError as e:
            print(f"发送失败: {e}")

    def send_initialization(self):
        """发送初始化命令（连续两次FF FF FF FF FF FF FF FD）"""
        init_data = bytes.fromhex("FF FF FF FF FF FF FF FD")
        for _ in range(2):  # 发送两次初始化命令
            message = can.Message(
                arbitration_id=0x1,  # CAN ID = 1
                data=init_data,
                is_extended_id=False
            )
            try:
                self.bus.send(message)
                print(f"成功发送初始化指令: {init_data.hex().upper()}")
                time.sleep(0.05)  # 50ms间隔
            except can.CanError as e:
                print(f"初始化发送失败: {e}")

    def receive_loop(self):
        """增强型接收循环"""
        print("启动接收监听...")
        while True:
            msg = self.bus.recv(timeout=2)  # 增加超时时间
            if msg:
                print(f"收到原始数据: ID=0x{msg.arbitration_id:X}, "
                      f"数据: {msg.data.hex().upper()}, "
                      f"时间戳: {msg.timestamp}")
            else:
                print("等待数据中...")

    def receive_response(self):
        """接收响应数据（带增强解析）"""
        print("启动响应监听...")
        start_time = time.time()
        while time.time() - start_time < 5:  # 5秒监听窗口
            msg = self.bus.recv(timeout=1)
            if msg:
                if len(msg.data) == 6:  # 确认6字节响应
                    print(f"收到有效响应: ID=0x{msg.arbitration_id:X}")
                    print(f"原始数据: {msg.data.hex().upper()}")
                    self._parse_response(msg.data)
                else:
                    print(f"收到非常规数据: {msg.data.hex().upper()}")
            else:
                print("等待响应中...")

    def _parse_response(self, data):
        """解析6字节响应数据"""
        # 根据上位机数据格式解析（示例数据：01 E3 65 7F F7 FC）
        status = {
            'byte1': data[0],
            'byte2': data[1],
            'byte3': data[2],
            'byte4': data[3],
            'byte5': data[4],
            'byte6': data[5]
        }
        print(f"解析结果: {status}")


# 使用示例
if __name__ == "__main__":
    # 初始化控制器（关键参数配置）
    mc = MotorController(
        interface='pcan',          # 硬件接口类型
        can_channel='PCAN_USBBUS1', # 根据实际设备修改
        bitrate=1000000            # 1Mbps波特率
    )

    # 执行初始化序列
    mc.send_initialization()

    # 启动响应接收
    mc.receive_response()


