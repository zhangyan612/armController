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
    encoder = ArduinoEncoder('COM21')

    # 需要查询的通道
    query_list = [
        (70, 7),
        (70, 8),
        (71, 6),
        (71, 8),
        (72, 4)
    ]

    result = encoder.query_batch(query_list)
    print(result)
