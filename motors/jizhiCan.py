import can

class MotorController:
    def __init__(self, interface='socketcan', can_channel='can0', bitrate=100000):
        # CAN总线初始化
        self.bus = can.interface.Bus(
            interface = interface,
            channel=can_channel,
            # bustype='socketcan',
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

    def receive_loop(self):
        """持续接收数据的循环"""
        while True:
            msg = self.bus.recv(1)  # 1秒超时
            if msg is not None:
                try:
                    data = self.parse_rx_data(msg.data)
                    print(f"收到电机反馈 - 位置: {data['position']}, "
                          f"速度: {data['velocity']}, "
                          f"电流: {data['current']}")
                except Exception as e:
                    print(f"数据解析错误: {e}")

# 使用示例
if __name__ == "__main__":
    # 初始化控制器
    mc = MotorController(can_channel='can0')
    
    # 发送位置控制命令（示例参数）
    mc.send_position_command(
        can_id=0x101,            # 目标CAN ID
        pos=0x8000,              # 位置基准点
        vel=0x900,               # 速度设置
        Kp=0x800,                # 位置环Kp
        Kd=0x800,                # 位置环Kd
        current=0x900            # 力矩设置
    )
    
    # 启动接收循环（需要在另一个线程运行）
    # mc.receive_loop()