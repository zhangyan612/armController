import serial
import time
import json

class ArduinoEncoder:
    def __init__(self, port='COM21', baudrate=115200, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout, dsrdtr=False, rtscts=False)
        # 等待 Arduino 输出 Ready
        while True:
            line = self.ser.readline().decode().strip()
            if line == "Ready":
                break
        self.ser.reset_input_buffer()

    def query_batch(self, pairs):
        """
        pairs: list of tuples (board, port)
        returns: list of dicts [{"board":70,"port":7,"angle":123.45}, ...]
        """
        if not pairs:
            return []

        # 构造命令
        cmd = "B"
        for board, port in pairs:
            cmd += f",{board},{port}"
        cmd += "\n"

        # 发送命令
        self.ser.write(cmd.encode())

        # 读取返回
        resp = self.ser.readline().decode().strip()
        try:
            data = json.loads(resp)
        except:
            data = []
        return data

# ----------------- 使用示例 -----------------
if __name__ == "__main__":
    port = '/dev/ttyUSB0'
    encoder = ArduinoEncoder(port)

    # 需要查询的通道
    query_list = [
        (70, 3), # Left shoulder
        (70, 5), # right shoulder
        (71, 1), # left arm wrist 1
        (71, 7), # left arm wrist 2
        (71, 8), # left arm elbow
        (72, 4), # right arm elbow
        (72, 6), # right arm wrist 1
        (72, 7), # right arm wrist 1
    ]

    result = encoder.query_batch(query_list)
    print(result)
