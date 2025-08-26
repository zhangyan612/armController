import serial
import time
import struct
from typing import List


class RobStrideMotor:
    """
    RobStride 电机控制类 - 兼容 motor1, motor2, motor3, motor4
    自动处理 CAN ID，支持速度模式
    """

    def __init__(self, ser: serial.Serial, motor_id: int):
        """
        初始化 RobStride 电机
        Args:
            ser: 串口对象
            motor_id: 电机ID (直接写真实ID，比如 motor2=0x14)
        """
        self.ser = ser
        self.motor_id = motor_id

    def _send_command(self, cmd: str):
        """
        发送十六进制命令并打印
        """
        frame = bytes.fromhex(cmd)
        self.ser.write(frame)
        print(f"Sent: {cmd}")
        time.sleep(0.05)

        # 接收响应
        response = self.ser.read_all()
        if response:
            print(f"Received: {response.hex()}")
        return response

    def initialize_adapter(self):
        """
        初始化 USB-CAN 适配器
        """
        print("Initializing adapter...")
        self._send_command("41 54 2b 41 54 0d 0a")
        time.sleep(0.2)

    def setup_motor(self):
        """
        发送初始化电机命令（对应第二套代码的序列）
        """
        print(f"Setting up motor ID {self.motor_id}...")
        # 设置电机参数
        cmd1 = f"41 54 00 07 e8 {self.motor_id:02x} 01 00 0d 0a"
        cmd2 = f"41 54 20 07 e8 {self.motor_id:02x} 08 00 c4 00 00 00 00 00 00 0d 0a"
        self._send_command(cmd1)
        time.sleep(0.1)
        self._send_command(cmd2)
        time.sleep(0.1)

    def set_velocity_mode(self):
        """
        设置速度模式
        """
        print(f"Setting velocity mode for motor ID {self.motor_id}...")
        cmd = f"41 54 90 07 e8 {self.motor_id:02x} 08 05 70 00 00 02 00 00 00 0d 0a"
        self._send_command(cmd)
        time.sleep(0.2)

    def enable_motor(self):
        """
        使能电机
        """
        print(f"Enabling motor ID {self.motor_id}...")
        cmd = f"41 54 18 07 e8 {self.motor_id:02x} 08 00 00 00 00 00 00 00 00 0d 0a"
        self._send_command(cmd)
        time.sleep(0.5)

    def disable_motor(self):
        """
        禁用电机
        """
        print(f"Disabling motor ID {self.motor_id}...")
        cmd = f"41 54 20 07 e8 {self.motor_id:02x} 08 a8 1c 3b 4e 84 11 b0 08 0d 0a"
        self._send_command(cmd)
        time.sleep(0.5)

    def _float_to_3bytes_from_le_tail(self, value: float) -> str:
        """
        协议要求：float 小端打包后取最后3字节
        例如：2.0 -> '00 00 40'
        """
        b = struct.pack('<f', value)
        tail3 = b[1:]
        return ' '.join(f"{x:02x}" for x in tail3)

    def set_velocity(self, velocity: float):
        """
        设置速度
        """
        val_hex = self._float_to_3bytes_from_le_tail(velocity)
        cmd = f"41 54 90 07 e8 {self.motor_id:02x} 08 24 70 00 00 00 {val_hex} 0d 0a"
        self._send_command(cmd)
        time.sleep(0.1)

    def set_acceleration(self, acceleration: float):
        """
        设置加速度
        """
        val_hex = self._float_to_3bytes_from_le_tail(acceleration)
        cmd = f"41 54 90 07 e8 {self.motor_id:02x} 08 25 70 00 00 00 {val_hex} 0d 0a"
        self._send_command(cmd)
        time.sleep(0.1)

    def move_at_velocity(self, velocity: float, acceleration: float):
        """
        在速度模式下移动
        """
        print(f"Motor {self.motor_id}: Moving at velocity={velocity}, acceleration={acceleration}")
        self.set_velocity(velocity)
        self.set_acceleration(acceleration)


class RobStrideController:
    """
    多电机控制器，支持 motor1、motor2、motor3、motor4
    """

    def __init__(self, port: str, baudrate: int = 921600):
        """
        初始化控制器
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        self.motors = {}

    def add_motor(self, motor_id: int):
        """
        添加一个电机
        """
        motor = RobStrideMotor(self.ser, motor_id)
        self.motors[motor_id] = motor
        return motor

    def close(self):
        """
        关闭串口
        """
        self.ser.close()


def main():
    """
    主函数：初始化并控制 motor1、motor2、motor3、motor4
    """
    print("Starting multi-motor velocity control...")

    # 初始化控制器
    controller = RobStrideController(port="COM21")

    # 添加电机（示例，motor2=0x14）
    motor1 = controller.add_motor(0x01)  # motor1
    motor2 = controller.add_motor(0x14)  # motor2
    motor3 = controller.add_motor(0x17)  # motor3
    motor4 = controller.add_motor(0x1A)  # motor4

    try:
        # 初始化适配器
        motor1.initialize_adapter()

        # 初始化所有电机
        for m in [motor1, motor2, motor3, motor4]:
            m.setup_motor()
            m.set_velocity_mode()
            m.enable_motor()

        # 示例：让所有电机以速度 3.0 rad/s，加速度 6.0 rad/s² 旋转
        velocity = 3.0
        acceleration = 6.0
        for m in [motor1, motor2, motor3, motor4]:
            m.move_at_velocity(velocity, acceleration)

        time.sleep(3)

        # 修改参数：加速到 5.0 rad/s
        velocity = 5.0
        acceleration = 10.0
        for m in [motor1, motor2, motor3, motor4]:
            m.move_at_velocity(velocity, acceleration)

        time.sleep(5)

        # 禁用所有电机
        for m in [motor1, motor2, motor3, motor4]:
            m.disable_motor()

    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        controller.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
