import time
import serial

class MotorController:
    def __init__(self, port='COM10', baudrate=115200):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        if self.ser.isOpen():
            print("Serial port is open")
        else:
            raise IOError("Failed to open serial port")

    def construct_message(self, id_value, pwm_value, time_value):
        if not (isinstance(pwm_value, int) and isinstance(time_value, int)):
            raise ValueError("PWM and Time values must be integers.")
        pwm_str = f"{pwm_value:04}"
        time_str = f"{time_value:05}"
        return f"#{id_value}#{pwm_str}#{time_str}#".encode('utf-8')

    def move_motor(self, id, pwm, duration):
        limitPWMList = {'03', '06'}
        if id not in limitPWMList:
            pwm = max(1580, min(pwm, 2420))

        msg = self.construct_message(id, pwm, duration)
        print(f"Sending: {msg}")
        self.ser.write(msg)
        time.sleep(0.01)

    def close(self):
        self.ser.close()

