import serial
import struct
import time

class MotorControl:
    def __init__(self, port='COM8', baudrate=19200, device_id=1, timeout=0.5):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.device_id = device_id

    def close(self):
        """关闭串口连接"""
        self.ser.close()

    def _crc16(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        le = crc.to_bytes(2, 'little')
        return le[::-1]  # 高字节在前

    def _build_frame(self, func: int, reg_addr: int, payload: bytes) -> bytes:
        frame = bytearray([self.device_id, func])
        frame += reg_addr.to_bytes(2, 'big')
        frame += payload
        frame += self._crc16(frame)
        return bytes(frame)

    def _send_recv(self, frame: bytes, resp_len: int) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        time.sleep(0.05)
        return self.ser.read(resp_len)

    # 通用读写函数
    def read32(self, reg_addr: int) -> int:
        """读取 32-bit 寄存器"""
        func = 0x03
        payload = (2).to_bytes(4, 'big')
        frame = self._build_frame(func, reg_addr, payload)
        expected = 1+1+2+4+2
        resp = self._send_recv(frame, expected)
        data = resp[4:8]
        return int.from_bytes(data, 'big', signed=True)

    def write32(self, reg_addr: int, value: int) -> None:
        """写入 32-bit 寄存器"""
        func = 0x06
        payload = struct.pack('>i', value)
        frame = self._build_frame(func, reg_addr, payload)
        expected = 1+1+2+4+2
        resp = self._send_recv(frame, expected)

    # 各功能函数
    def version_info(self, enable: bool = True) -> int:
        """寄存器 0x0000: 版本信息，0/1"""
        if enable in (True, False):
            self.write32(0x0000, int(enable))
        return self.read32(0x0000)

    def set_baudrate(self, mode: int) -> int:
        """寄存器 0x0001: 通信波特率 1-5"""
        self.write32(0x0001, mode)
        return self.read32(0x0001)

    def set_address(self, addr: int) -> int:
        """寄存器 0x0002: 设备地址 1-247"""
        self.write32(0x0002, addr)
        return self.read32(0x0002)

    def read_temperature(self) -> int:
        """寄存器 0x0003: 电机温度（℃）"""
        return self.read32(0x0003)

    def servo_status(self) -> int:
        """寄存器 0x0010: 伺服状态 0/1"""
        return self.read32(0x0010)

    def position1(self) -> int:
        """寄存器 0x0013: 编码器位置(1)"""
        return self.read32(0x0013)

    def speed(self) -> int:
        """寄存器 0x0014: 电机转速（rpm）"""
        return self.read32(0x0014)

    def position2(self) -> int:
        """寄存器 0x0015: 编码器位置(2)"""
        return self.read32(0x0015)

    def current(self) -> int:
        """寄存器 0x0019: 驱动电流（mA）"""
        return self.read32(0x0019)

    def run_status(self) -> int:
        """寄存器 0x001A: 运行状态 0/1"""
        return self.read32(0x001A)

    def fault_code(self) -> int:
        """寄存器 0x001B: 故障代码 0-3"""
        return self.read32(0x001B)

    def set_max_current(self, ma: int) -> int:
        """寄存器 0x001E: 电机上限电流（mA）"""
        self.write32(0x001E, ma)
        return self.read32(0x001E)

    def set_pid(self, kp: int, ki: int, kd: int):
        """寄存器 0x0020-0x0022: 位置环 Kp, Ki, Kd"""
        self.write32(0x0020, kp)
        self.write32(0x0021, ki)
        self.write32(0x0022, kd)
        return (self.read32(0x0020), self.read32(0x0021), self.read32(0x0022))

    def set_accel(self, ms: int) -> int:
        """寄存器 0x0027: 加速度（ms）"""
        self.write32(0x0027, ms)
        return self.read32(0x0027)

    def set_decel(self, ms: int) -> int:
        """寄存器 0x0028: 减速度（ms）"""
        self.write32(0x0028, ms)
        return self.read32(0x0028)

    def save_params(self) -> None:
        """寄存器 0x002D: 保存参数"""
        self.write32(0x002D, 1)

    def set_target_speed(self, rpm: int) -> int:
        """寄存器 0x002E: 目标速度（rpm）"""
        self.write32(0x002E, rpm)
        return self.read32(0x002E)

    def set_speed_mode(self, rpm: int) -> int:
        """寄存器 0x002F: 速度模式（rpm）"""
        self.write32(0x002F, rpm)
        return self.read32(0x002F)

    def set_current_mode(self, val: int) -> int:
        """寄存器 0x0030: 电流模式"""
        self.write32(0x0030, val)
        return self.read32(0x0030)

    def set_origin(self) -> None:
        """寄存器 0x0031: 设置原点"""
        self.write32(0x0031, 1)

    def return_to_origin(self) -> None:
        """寄存器 0x0032: 返回原点"""
        self.write32(0x0032, 1)

    def stop(self) -> None:
        """寄存器 0x0033: 停止电机"""
        self.write32(0x0033, 1)

    def move_position(self, pos: int) -> int:
        """寄存器 0x0081: 位置模式运行（无减速）"""
        self.write32(0x0081, pos)
        return pos

# 测试代码
if __name__ == '__main__':
    mc = MotorControl(port='COM8', baudrate=19200, device_id=0x01)
    try:
        print("Version Info:", mc.version_info())
        # print("Set Baudrate to 2:", mc.set_baudrate(2))
        # print("Set Address to 1:", mc.set_address(1))
        print("Temperature:", mc.read_temperature())
        print("Servo Status:", mc.servo_status())
        print("Position1:", mc.position1())
        print("Speed:", mc.speed())
        print("Position2:", mc.position2())
        print("Current:", mc.current())
        print("Run Status:", mc.run_status())
        print("Fault Code:", mc.fault_code())
        # print("Set Max Current:", mc.set_max_current(1000))
        # print("Set PID:", mc.set_pid(100, 10, 1))
        # print("Set Accel:", mc.set_accel(200))
        # print("Set Decel:", mc.set_decel(200))
        # mc.save_params()
        # print("Set Target Speed:", mc.set_target_speed(1000))
        # print("Set Speed Mode:", mc.set_speed_mode(500))
        # print("Set Current Mode:", mc.set_current_mode(100))
        mc.set_origin()
        mc.return_to_origin()


        # 伺服开/关测试
        print("Disabling servo...")
        mc.write32(0x0010, 0)
        print("Servo Status after disable:", mc.servo_status())
        print("Enabling servo...")
        mc.write32(0x0010, 1)
        print("Servo Status after enable:", mc.servo_status())

        # 编码器位置测试
        print("Encoder1 Position:", mc.position1())
        print("Encoder2 Position:", mc.position2())

        # 其他功能测试示例
        mc.move_position(0)
        time.sleep(1)

        # 伺服开/关测试
        mc.write32(0x0010, 0)

        mc.stop()

    finally:
        mc.close()
