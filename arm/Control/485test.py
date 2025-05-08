import serial
import struct
import time

class MotorControl:
    def __init__(self, port='COM8', baudrate=19200, device_id=1, timeout=0.5):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.device_id = device_id

    def close(self):
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
        print("► TX:", frame.hex().upper())
        self.ser.write(frame)
        time.sleep(0.05)
        resp = self.ser.read(resp_len)
        print("◄ RX:", resp.hex().upper())
        return resp

    def test_connection(self) -> bool:
        func = 0x03; reg = 0x0010; count = 2
        payload = count.to_bytes(4, 'big')       # 00 00 00 02
        frame = self._build_frame(func, reg, payload)
        expected = 1+1+2+4+2                   # 10 bytes
        resp = self._send_recv(frame, expected)
        if len(resp) != expected or resp[1] != func:
            raise IOError("Connection test failed or bad response")
        return True

    def read_register(self, reg_addr: int) -> int:
        """
        读 32-bit 寄存器，返回带符号整数
        响应帧：Addr(1) Func(1) RegAddr(2) Data(4) CRC(2)
        """
        func = 0x03
        payload = (2).to_bytes(4, 'big')
        frame = self._build_frame(func, reg_addr, payload)
        expected = 1+1+2+4+2
        resp = self._send_recv(frame, expected)
        if len(resp) != expected or resp[1] != func:
            raise IOError(f"Bad read response: {resp.hex().upper()}")

        # ***关键修正***：数据从索引4开始，长度4字节
        data_bytes = resp[4:8]
        return int.from_bytes(data_bytes, 'big', signed=True)

    def write_register(self, reg_addr: int, value: int) -> None:
        func = 0x06
        payload = struct.pack('>i', value)
        frame = self._build_frame(func, reg_addr, payload)
        expected = 1+1+2+4+2
        resp = self._send_recv(frame, expected)
        if len(resp) != expected or resp[1] != func:
            raise IOError(f"Bad write response: {resp.hex().upper()}")

    def enable(self):
        print("==> Testing connection …")
        self.test_connection()
        print("==> Enabling servo …")
        self.write_register(0x0010, 1)
        status = self.read_register(0x0010)
        if status != 1:
            raise RuntimeError(f"Enable failed, got {status}")
        print("✔ Servo enabled.")

    def move_to(self, position: int):
        if self.read_register(0x0010) != 1:
            self.enable()
        print(f"==> Moving to {position} …")
        self.write_register(0x0082, position)


if __name__ == '__main__':
    mc = MotorControl(port='COM8', baudrate=19200, device_id=0x01)
    try:
        mc.enable()
        mc.move_to(20000)
        time.sleep(1)
        mc.move_to(-20000)
    finally:
        mc.close()
