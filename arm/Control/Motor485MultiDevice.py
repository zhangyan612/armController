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
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        le = crc.to_bytes(2, 'little')
        return le[::-1]

    def _build_frame(self, func: int, reg_addr: int, payload: bytes, device_id: int = None) -> bytes:
        dev_id = device_id if device_id is not None else self.device_id
        frame = bytearray([dev_id, func])
        frame += reg_addr.to_bytes(2, 'big')
        frame += payload
        frame += self._crc16(frame)
        return bytes(frame)

    def _send_recv(self, frame: bytes, resp_len: int) -> bytes:
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        time.sleep(0.05)
        return self.ser.read(resp_len)

    def read32(self, reg_addr: int, device_id: int = None) -> int:
        func = 0x03
        payload = (2).to_bytes(4, 'big')
        frame = self._build_frame(func, reg_addr, payload, device_id)
        expected = 1 + 1 + 2 + 4 + 2
        resp = self._send_recv(frame, expected)
        data = resp[4:8]
        return int.from_bytes(data, 'big', signed=True)

    def write32(self, reg_addr: int, value: int, device_id: int = None) -> None:
        func = 0x06
        payload = struct.pack('>i', value)
        frame = self._build_frame(func, reg_addr, payload, device_id)
        expected = 1 + 1 + 2 + 4 + 2
        self._send_recv(frame, expected)

    # Functional Methods (All with optional device_id)
    def version_info(self, enable: bool = True, device_id: int = None) -> int:
        if enable in (True, False):
            self.write32(0x0000, int(enable), device_id)
        return self.read32(0x0000, device_id)

    def set_baudrate(self, mode: int, device_id: int = None) -> int:
        self.write32(0x0001, mode, device_id)
        return self.read32(0x0001, device_id)

    def set_address(self, addr: int, device_id: int = None) -> int:
        self.write32(0x0002, addr, device_id)
        return self.read32(0x0002, device_id)

    def read_temperature(self, device_id: int = None) -> int:
        return self.read32(0x0003, device_id)

    def servo_status(self, device_id: int = None) -> int:
        return self.read32(0x0010, device_id)

    def position1(self, device_id: int = None) -> int:
        return self.read32(0x0013, device_id)

    def speed(self, device_id: int = None) -> int:
        return self.read32(0x0014, device_id)

    def position2(self, device_id: int = None) -> int:
        return self.read32(0x0015, device_id)

    def current(self, device_id: int = None) -> int:
        return self.read32(0x0019, device_id)

    def run_status(self, device_id: int = None) -> int:
        return self.read32(0x001A, device_id)

    def fault_code(self, device_id: int = None) -> int:
        return self.read32(0x001B, device_id)

    def set_max_current(self, ma: int, device_id: int = None) -> int:
        self.write32(0x001E, ma, device_id)
        return self.read32(0x001E, device_id)

    def set_pid(self, kp: int, ki: int, kd: int, device_id: int = None):
        self.write32(0x0020, kp, device_id)
        self.write32(0x0021, ki, device_id)
        self.write32(0x0022, kd, device_id)
        return (
            self.read32(0x0020, device_id),
            self.read32(0x0021, device_id),
            self.read32(0x0022, device_id)
        )

    def set_accel(self, ms: int, device_id: int = None) -> int:
        self.write32(0x0027, ms, device_id)
        return self.read32(0x0027, device_id)

    def set_decel(self, ms: int, device_id: int = None) -> int:
        self.write32(0x0028, ms, device_id)
        return self.read32(0x0028, device_id)

    def save_params(self, device_id: int = None) -> None:
        self.write32(0x002D, 1, device_id)

    def set_target_speed(self, rpm: int, device_id: int = None) -> int:
        self.write32(0x002E, rpm, device_id)
        return self.read32(0x002E, device_id)

    def set_speed_mode(self, rpm: int, device_id: int = None) -> int:
        self.write32(0x002F, rpm, device_id)
        return self.read32(0x002F, device_id)

    def set_current_mode(self, val: int, device_id: int = None) -> int:
        self.write32(0x0030, val, device_id)
        return self.read32(0x0030, device_id)

    def set_origin(self, device_id: int = None) -> None:
        self.write32(0x0031, 1, device_id)

    def return_to_origin(self, device_id: int = None) -> None:
        self.write32(0x0032, 1, device_id)

    def stop(self, device_id: int = None) -> None:
        self.write32(0x0033, 1, device_id)

    def move_position(self, pos: int, device_id: int = None) -> int:
        self.write32(0x0081, pos, device_id)
        return pos

    def move_accel(self, pos: int, device_id: int = None) -> int:
        self.write32(0x0082, pos, device_id)
        return pos

    def enable(self, device_id: int = None):
        self.write32(0x0010, 1, device_id)

    def disable(self, device_id: int = None):
        self.write32(0x0010, 0, device_id)


# Example usage:
if __name__ == '__main__':
    # mc = MotorControl(port='COM3', baudrate=19200)

    # linux lsusb
    mc = MotorControl(port='/dev/arm1', baudrate=19200)

    # permission deny 
    # sudo usermod -aG dialout $USER
    # log out and log in
    # group command to see list

    # To make the permission change permanent, you can create a udev rule:
    # sudo nano /etc/udev/rules.d/50-myusb.rules
    # Add this line:
    # KERNEL=="ttyUSB[0-9]*", MODE="0666"
    # Then reload udev rules:
    # sudo udevadm control --reload-rules
    # sudo udevadm trigger
    # This will make all USB serial devices accessible to all users.

    try:
        # for dev_id in range(1, 7):
        dev_id = 6
        print(f"[Device {dev_id}] Version Info:", mc.version_info(device_id=dev_id))
        print(f"[Device {dev_id}] Temperature:", mc.read_temperature(device_id=dev_id))
        print(f"[Device {dev_id}] Servo Status:", mc.servo_status(device_id=dev_id))
        mc.disable(device_id=dev_id)
        print(f"[Device {dev_id}] Disabled.")
        mc.enable(device_id=dev_id)
        print(f"[Device {dev_id}] Enabled.")


        print("Setting target speed to 1000 RPM...", mc.set_target_speed(1000, dev_id))
        print("Setting accel to 500 ms...", mc.set_accel(500, dev_id))
        print("Setting decel to 500 ms...", mc.set_decel(500, dev_id))
        time.sleep(1)

        current_pos = mc.position1(dev_id)
        print(f'current position {current_pos}')

        print("Moving with accel to 20000...", mc.move_accel(327680, dev_id))

        time.sleep(2)
        print("Moving with accel to 20000...", mc.move_accel(0, dev_id))

        time.sleep(1)


        mc.disable(device_id=dev_id)
        print(f"[Device {dev_id}] Disabled.")

    finally:
        mc.close()
