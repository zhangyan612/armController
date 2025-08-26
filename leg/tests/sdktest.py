import serial
import time

class MotorController:
    def __init__(self, port="COM37", baudrate=115200, timeout=0.5):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    def build_command(self, comm_type, motor_id, data_bytes):
        """
        构建兼容所有电机的 CAN 帧
        comm_type: 通信类型 (1 byte)
        motor_id: 电机ID (1~4)
        data_bytes: 数据域
        """
        # 计算 node_id: 电机ID << 2
        node_id = motor_id << 2

        # 构建扩展ID部分
        ext_id_bytes = bytes([comm_type, 0x07, 0xE8, node_id])

        # 数据长度
        data_len = bytes([len(data_bytes)])

        # 完整帧 = 帧头(41 54) + 扩展ID + 数据长度 + 数据 + 帧尾(0D 0A)
        frame = b"\x41\x54" + ext_id_bytes + data_len + data_bytes + b"\x0D\x0A"
        return frame

    def send_command(self, comm_type, motor_id, data_bytes):
        frame = self.build_command(comm_type, motor_id, data_bytes)
        self.ser.write(frame)
        print(f"Send Motor[{motor_id}] Frame:", frame.hex(" ").upper())

    def set_velocity(self, motor_id, velocity, acceleration):
        """
        设置速度模式
        velocity: 目标速度 (float)
        acceleration: 加速度 (float)
        """
        # 协议要求: velocity 和 acceleration 转成 uint16，小端模式
        vel_val = int(velocity * 1000)  # 假设单位 m/s -> mm/s
        acc_val = int(acceleration * 1000)

        vel_bytes = vel_val.to_bytes(2, byteorder='little', signed=True)
        acc_bytes = acc_val.to_bytes(2, byteorder='little', signed=False)

        # 构建数据域
        data_bytes = vel_bytes + acc_bytes

        # 发送，comm_type 根据协议定义，比如 0x90
        self.send_command(0x90, motor_id, data_bytes)

    def move_all_motors(self, velocities, acceleration):
        """
        一次性设置 1-4 号电机的速度
        velocities: [v1, v2, v3, v4]
        acceleration: 加速度 (相同)
        """
        for i, vel in enumerate(velocities, start=1):
            self.set_velocity(i, vel, acceleration)
            time.sleep(0.02)  # 防止总线冲突

    def close(self):
        self.ser.close()


# ===== 测试代码 =====
if __name__ == "__main__":
    mc = MotorController("COM21", 921600)

    # 比如想让 motor1=1.0, motor2=2.0, motor3=-1.5, motor4=0.5
    mc.move_all_motors([1.0, 2.0, -1.5, 0.5], acceleration=5.0)

    mc.close()
