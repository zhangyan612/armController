# motor_control.py
import serial
import struct
import time

class MotorControl:
    def __init__(self, port='COM8', baudrate=19200, device_id=1, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.device_id = device_id
        self.timeout = timeout
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _crc16(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, 'little')

    def _build_frame(self, func: int, reg_addr: int, payload: bytes) -> bytes:
        frame = bytearray([self.device_id, func])
        frame += reg_addr.to_bytes(2, 'big')
        frame += payload
        frame += self._crc16(frame)
        return bytes(frame)

    def _send(self, frame: bytes, resp_len: int) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        time.sleep(0.05)
        return self.ser.read(resp_len)

    def read_register(self, reg_addr: int, count: int = 2) -> bytes:
        """
        读 32-bit 寄存器（跨两个 16-bit 寄存器）。
        - func=0x03
        - count=2
        返回 raw bytes (4 bytes data)
        """
        func = 0x03
        payload = count.to_bytes(2, 'big')
        frame = self._build_frame(func, reg_addr, payload)
        # resp: id(1)+func(1)+bytecnt(1)+data(2*count)+crc(2)
        expected = 1 + 1 + 1 + 2*count + 2
        resp = self._send(frame, expected)
        if len(resp) != expected or resp[1] != func:
            raise IOError(f"Bad read response: {resp.hex().upper()}")
        # data bytes start at offset 3
        return resp[3:3+2*count]

    def write_register(self, reg_addr: int, value: int) -> None:
        """
        写 32-bit 寄存器（跨两个 16-bit 寄存器）。
        - func = 0x06
        - payload = 4 bytes big-endian signed
        """
        func = 0x06
        payload = struct.pack('>i', value)
        frame = self._build_frame(func, reg_addr, payload)
        # resp echoes: id+func+addr(2)+payload(4)+crc(2)
        expected = 1 + 1 + 2 + 4 + 2
        resp = self._send(frame, expected)
        if len(resp) != expected or resp[1] != func:
            raise IOError(f"Bad write response: {resp.hex().upper()}")

    def enable(self):
        """使能伺服 (寄存器 0x0010, 写入 1)。"""
        # 写 4 字节
        self.write_register(0x0010, 1)
        # 读回确认
        data = self.read_register(0x0010)
        status = int.from_bytes(data, 'big', signed=False)
        if status != 1:
            raise RuntimeError(f"Enable failed, got {status}")
        print("✔ Servo enabled.")

    def move_to(self, position: int):
        """
        位置模式移动 (寄存器 0x0082, 32-bit signed)
        """
        # 确保已使能
        data = self.read_register(0x0010)
        if int.from_bytes(data, 'big') != 1:
            self.enable()
        # 写入目标位置
        self.write_register(0x0082, position)
        print(f"► Moving to {position}")

# Usage example
if __name__ == '__main__':
    mc = MotorControl(port='COM8', baudrate=19200, device_id=0x01)
    try:
        mc.enable()
        mc.move_to(20000)
        time.sleep(1)
        mc.move_to(-20000)
    finally:
        mc.close()
