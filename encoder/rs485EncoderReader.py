import serial
import time

class EncoderReader:
    def __init__(self, port='COM5', baudrate=115200, timeout=1):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout
        )

    def close(self):
        self.ser.close()

    def calculate_crc(self, cf, sf, df0, df1):
        return cf ^ sf ^ df0 ^ df1

    def send_command(self, command, repeat=1, delay=0.0008):
        for _ in range(repeat):
            self.ser.write(command)
            time.sleep(delay)

    def read_position(self):
        self.send_command(b'\x02')
        response = self.ser.read(5)
        if len(response) == 5:
            cf, sf, df0, df1, crc = response
            if crc == self.calculate_crc(cf, sf, df0, df1):
                angle = df0 + (df1 << 8)
                return angle, sf
        return None, None

    def calibrate(self):
        for _ in range(10):
            self.send_command(b'\xBA')
            response = self.ser.read(5)
            if len(response) == 5:
                cf, sf, df0, df1, crc = response
                if crc == self.calculate_crc(cf, sf, df0, df1):
                    angle = df0 + (df1 << 8)
                    print(f"校准中，角度: {angle}, 运行状态: {sf}")
            time.sleep(0.0008)

        print("校准初始化...")
        time.sleep(1)

        while True:
            angle, status = self.read_position()
            if status is not None:
                print(f"角度: {angle}, 运行状态: {status}")
                if status == 0x08:
                    print("校准结束")
                    break

    def reset_position(self):
        for _ in range(10):
            self.send_command(b'\xC2')
            response = self.ser.read(5)
            if len(response) == 5:
                cf, sf, df0, df1, crc = response
                if crc == self.calculate_crc(cf, sf, df0, df1):
                    angle = df0 + (df1 << 8)
                    print(f"清零中，角度: {angle}, 运行状态: {sf}")
            time.sleep(0.0007)
        print("清零操作完成")

    def get_position(self):
        angle, status = self.read_position()
        if angle is not None and status is not None:
            return angle, status
        return None, None

    def continuous_output(self):
        try:
            while True:
                angle, status = self.read_position()
                if angle is not None and status is not None:
                    print(f"角度: {angle}, 运行状态: {status}")
                    if status == 0x08:
                        return angle
        except KeyboardInterrupt:
            print("停止输出位置信号")


# Example usage
if __name__ == "__main__":
    encoder = EncoderReader(port='COM5')
    try:
        angle, status = encoder.get_position()
        print(f"Encoder value: {angle}, Status: {status}")
    finally:
        encoder.close()
