import time
import serial


class ServoController:
    def __init__(self, port='COM10', baudrate=115200, timeout=1):
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=timeout
            )
            if self.ser.isOpen():
                print(f"✅ Serial port {port} opened successfully.")
        except serial.SerialException as e:
            print(f"❌ Error opening serial port: {e}")
            raise

    def _format_id(self, id):
        return f"{int(id):03d}"

    def _format_value(self, value, width=4):
        return f"{int(value):0{width}d}"

    def send_command(self, command: str):
        full_cmd = command + '!'
        print(f"→ Sending: {full_cmd}")
        self.ser.write(full_cmd.encode())

    def move_servo(self, id, angle, duration):
        sid = self._format_id(id)
        pwm = self._format_value(angle)
        time_ms = self._format_value(duration)
        self.send_command(f"#{sid}P{pwm}T{time_ms}")

    def read_position(self, id):
        sid = self._format_id(id)
        self.send_command(f"#{sid}PRAD")
        time.sleep(0.01)
        if self.ser.in_waiting:
            data = self.ser.read(self.ser.in_waiting)
            print(f"← Raw response: {data}")
            try:
                response = data.decode()
                pos = int(response.strip('!').split('P')[1])
                return pos
            except (IndexError, ValueError):
                print("⚠️ Failed to parse position.")
        return None

    def release_torque(self, id):
        sid = self._format_id(id)
        self.send_command(f"#{sid}PULK")

    def restore_torque(self, id):
        sid = self._format_id(id)
        self.send_command(f"#{sid}PULR")

    def reset_to_center(self, id):
        sid = self._format_id(id)
        self.send_command(f"#{sid}PSCK")

    def close(self):
        if self.ser.isOpen():
            self.ser.close()
            print("🔌 Serial port closed.")


# Example usage
if __name__ == "__main__":
    # port = 'COM10'
    port = '/dev/servo1'

    servo = ServoController(port=port)
    # servo.reset_to_center(1)
    # time.sleep(1)
    # servo.reset_to_center(2)

    servo.restore_torque(1)
    servo.restore_torque(2)

    time.sleep(1)

    servo.move_servo(1, 850, 1000)  # center
    time.sleep(2)

    # servo.move_servo(1, 800, 1000)
    # time.sleep(2)
    # servo.move_servo(1, 1000, 2000)
    # time.sleep(2)

    servo.move_servo(2, 1000, 1000) # center
    time.sleep(2)


    # servo.move_servo(2, 900, 1000)
    # time.sleep(2)
    # servo.move_servo(2, 1000, 1000)
    # time.sleep(2)
    # servo.move_servo(2, 1100, 1000)
    # time.sleep(2)


    # servo.move_servo(2, 1050, 1000)
    # time.sleep(1)

    position = servo.read_position(1)
    print(f"📍 Current position of servo 1 : {position}")

    position = servo.read_position(2)
    print(f"📍 Current position of servo 2 : {position}")
    servo.release_torque(1)
    servo.release_torque(2)

    time.sleep(2)

    servo.close()
