import serial
import time
import os
import json
from threading import Thread, Lock

class MotorController:
    def __init__(self, port='COM6', baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.reduction_ratio = 1
        self.rotation_count = 0
        self.enabled = False
        self.running = True
        self.data_file = "motor_data.json"
        self.lock = Lock()  # 用于线程同步
        self.response_received = False  # 标记是否收到响应
        self.encoder_angle = 0  # 编码器角度

        self.load_q_value()
        self.receive_thread = Thread(target=self._receive_data)
        self.receive_thread.start()

    def load_q_value(self):
        if os.path.exists(self.data_file):
            with open(self.data_file, 'r') as f:
                data = json.load(f)
                self.rotation_count = data.get('q', 0)
                print(f"Loaded Q value: {self.rotation_count}")

    def save_q_value(self):
        with open(self.data_file, 'w') as f:
            json.dump({'q': self.rotation_count}, f)

    def send_command(self, cmd, retries=3, timeout=2.0):
        """发送命令并等待响应，支持重试"""
        for attempt in range(retries):
            with self.lock:
                self.response_received = False
                # 发送命令
                self.ser.write(f"{cmd}\r\n".encode())
                self.ser.flush()
                print(f"Sent: {cmd} (Attempt {attempt + 1}/{retries})")
            
            # 等待响应
            received = False
            start_time = time.time()
            while time.time() - start_time < timeout:
                with self.lock:
                    if self.response_received:
                        received = True
                        break
                time.sleep(0.01)
            
            if received:
                print(f"Command {cmd} confirmed")
                return True
            else:
                print(f"No response for {cmd}, retrying...")
        
        print(f"Failed after {retries} retries: {cmd}")
        return False

    def motor_enable(self, enable=True):
        """E命令-电机使能（带重试）"""
        cmd = f"E={1 if enable else 0};"
        if self.send_command(cmd, retries=3):
            self.enabled = enable
            print(f"Motor {'Enabled' if enable else 'Disabled'}")

    def degrees_to_value(self, degrees):
        return round(degrees * 6.28 / 360, 2)

    def set_angle(self, angle_deg):
        """A命令-转动指定角度（带重试）"""
        if not self.enabled:
            print("Motor not enabled!")
            return
        
        actual_turns = angle_deg / 360 * self.reduction_ratio
        rotate_value = self.degrees_to_value(angle_deg) * self.reduction_ratio
        sign = '+' if actual_turns >= 0 else '-'
        cmd = f"A={sign}{abs(rotate_value):07.3f};"
        if self.send_command(cmd, retries=3):
            self.rotation_count += actual_turns
            self.save_q_value()
            print(f"Rotated {actual_turns:.3f} turns")
        else:
            print("Failed to rotate")

    def set_reduction_ratio(self, ratio):
        """B命令-设置减速比（带重试）只在python端设置电机端不工作"""
        # cmd = f"B={max(1, int(ratio))};"
        # if self.send_command(cmd, retries=3):
        self.reduction_ratio = max(1, int(ratio))
        print(f"Reduction ratio set to {self.reduction_ratio}:1")

    def query_rotation(self):
        """发送Q=;命令查询圈数（带重试）"""
        cmd = "Q=;"
        if self.send_command(cmd, retries=3):
            with self.lock:
                return self.rotation_count
        return None

    # 新增专用查询方法（与协议严格匹配）
    def query_encoder_angle(self):
        """发送C;命令查询编码器原始值（带重试）"""
        cmd = "C;"
        if self.send_command(cmd, retries=3):
            with self.lock:
                return self.encoder_angle
        return None


    def _receive_data(self):
        """接收线程处理函数（支持C/Q命令专用格式）"""
        while self.running:
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline().decode().strip()
                print(f"Received raw: {raw_data}")  # 调试输出原始数据
                
                with self.lock:
                    self.response_received = True
                    
                    # 处理编码器角度响应 (C;命令)
                    if raw_data.lstrip('-').isdigit():  # 纯数字响应
                        try:
                            self.encoder_angle = int(raw_data)
                            print(f"Encoder angle updated: {self.encoder_angle}")
                        except ValueError:
                            print(f"Invalid C response: {raw_data}")
                    
                    # 处理圈数响应 (Q=命令)
                    elif raw_data.startswith('Q:'):
                        try:
                            self.rotation_count = int(raw_data.split(':')[1])
                            self.save_q_value()
                            print(f"Updated Q: {self.rotation_count}")
                        except (IndexError, ValueError):
                            print(f"Invalid Q response: {raw_data}")

    def close(self):
        self.running = False
        self.receive_thread.join()
        self.ser.close()
        print("Connection closed")

# 使用示例
if __name__ == "__main__":
    mc = MotorController(port='COM32')
    try:
        # mc.motor_enable(False) 
        # mc.set_reduction_ratio(80)

        # TODO: 设置关节限位
        # mc.set_angle(0)    # 度数 0 - 360
        # print("Current Q:", mc.query_rotation())
        # mc.set_angle(-540)    # 反转1.5圈
        a = mc.query_encoder_angle()
        print(a)

        # time.sleep(0.1)
        # a = mc.query_encoder_angle()
        # print(a)

        # time.sleep(1.5)
        # a = mc.query_encoder_angle()
        # print(a)

        # q = mc.query_rotation()
        # print(q)

    finally:
        mc.motor_enable(False)
        mc.close()













        

# [2025-08-12 22:08:26.776]# RECV HEX>
# 3C DB 76 7F DF FF 

# [2025-08-12 22:08:27.491]# SEND ASCII>
# C;



# [2025-08-12 22:08:28.547]# SEND ASCII>
# C;



# [2025-08-12 22:08:28.605]# RECV HEX>
# 41 6E 36 DF 9B BF 

# [2025-08-12 22:08:29.165]# SEND ASCII>
# C;



# [2025-08-12 22:08:29.219]# RECV HEX>
# 41 EE 3E BF FF 7F 7E 7D BB EF FF F7 3F EF 

# [2025-08-12 22:08:29.943]# SEND ASCII>
# C;



# [2025-08-12 22:08:29.996]# RECV HEX>
# 43 EF 3F FB B7 FF FB FF 7F E7 EF F7 EF 

# [2025-08-12 22:08:30.450]# SEND ASCII>
# C;



# [2025-08-12 22:08:30.514]# RECV HEX>
# 43 EF 76 

# [2025-08-12 22:08:31.659]# SEND ASCII>
# C;



# [2025-08-12 22:08:31.717]# RECV HEX>
# D8 FB 3F FF B7 77 EF 

# [2025-08-12 22:08:33.518]# SEND ASCII>
# C;



# [2025-08-12 22:08:33.579]# RECV HEX>
# 43 EF 3F FF FE BF AF 7F D7 FB B9 B7 3F 




# [2025-08-12 22:12:37.489]# SEND ASCII>
# C;



# [2025-08-12 22:12:37.542]# RECV HEX>
# 41 6E 67 6C 65 3A 20 2D 33 2E 31 36 37 38 36 34 20 72 61 64 2C 20 52 61 77 3A 20 2D 38 32 36 30 2C 20 52 6F 74 61 74 69 6F 6E 3A 20 30 0D 0A 

# [2025-08-12 22:12:38.857]# SEND ASCII>
# C;



# [2025-08-12 22:12:38.908]# RECV HEX>
# 41 6E 67 6C 65 3A 20 2D 33 2E 31 36 37 34 38 30 20 72 61 64 2C 20 52 61 77 3A 20 2D 38 32 35 39 2C 20 52 6F 74 61 74 69 6F 6E 3A 20 30 0D 0A 
