import serial
import time
import os
import json
from threading import Thread, Lock
import re

class MotorController:
    def __init__(self, port='COM6', baudrate=115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        self.reduction_ratio = 1
        self.rotation_count = 0
        self.enabled = False
        self.running = True
        self.data_file = "motor_data.json"
        self.lock = Lock()
        self.response_received = False
        self.encoder_angle = 0
        self.raw_encoder_value = 0
        self.last_response = ""
        
        self.load_q_value()
        self.receive_thread = Thread(target=self._receive_data)
        self.receive_thread.daemon = True
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

    def send_command(self, cmd, retries=3, timeout=0.5):
        """发送命令并等待响应，支持重试"""
        for attempt in range(retries):
            with self.lock:
                self.response_received = False
                self.last_response = ""
            
            # 清空输入缓冲区
            self.ser.reset_input_buffer()
            
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
                print(f"Command {cmd} confirmed with response: {self.last_response}")
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
            return True
        return False

    def degrees_to_value(self, degrees):
        return round(degrees * 6.28 / 360, 2)

    def set_angle(self, angle_deg):
        """A命令-转动指定角度（带重试）"""
        if not self.enabled:
            print("Motor not enabled!")
            return False
        
        rotate_value = self.degrees_to_value(angle_deg) * self.reduction_ratio
        sign = '+' if rotate_value >= 0 else '-'
        cmd = f"A={sign}{abs(rotate_value):07.3f};"
        if self.send_command(cmd, retries=3):
            actual_turns = angle_deg / 360 * self.reduction_ratio
            self.rotation_count += actual_turns
            self.save_q_value()
            print(f"Set angle to {rotate_value:.3f} rad")
            return True
        return False

    def set_reduction_ratio(self, ratio):
        """B命令-设置减速比（带重试）"""
        cmd = f"B={max(1, int(ratio))};"
        if self.send_command(cmd, retries=3):
            self.reduction_ratio = max(1, int(ratio))
            print(f"Reduction ratio set to {self.reduction_ratio}:1")
            return True
        return False

    def query_rotation(self):
        """发送Q=;命令查询圈数（带重试）"""
        cmd = "Q=;"
        if self.send_command(cmd, retries=3):
            # 尝试从响应中解析圈数
            if "Rotation:" in self.last_response:
                try:
                    match = re.search(r"Rotation:\s*(\d+)", self.last_response)
                    if match:
                        self.rotation_count = int(match.group(1))
                        print(f"Updated rotation count: {self.rotation_count}")
                except ValueError:
                    print("Could not parse rotation count from response")
            return self.rotation_count
        return None

    def query_encoder_angle(self):
        """发送C;命令查询编码器原始值（带重试）"""
        cmd = "C;"
        if self.send_command(cmd, retries=3):
            # 尝试从响应中解析编码器值
            if "Raw:" in self.last_response:
                try:
                    match = re.search(r"Raw:\s*(-?\d+)", self.last_response)
                    if match:
                        self.raw_encoder_value = int(match.group(1))
                        print(f"Updated raw encoder value: {self.raw_encoder_value}")
                except ValueError:
                    print("Could not parse raw encoder value from response")
            
            if "Angle:" in self.last_response:
                try:
                    match = re.search(r"Angle:\s*([\d.]+)\s*rad", self.last_response)
                    if match:
                        self.encoder_angle = float(match.group(1))
                        print(f"Updated encoder angle: {self.encoder_angle} rad")
                except ValueError:
                    print("Could not parse encoder angle from response")
            
            return self.raw_encoder_value
        return None

    def _receive_data(self):
        """接收线程处理函数"""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    raw_data = self.ser.readline().decode().strip()
                    if raw_data:  # 只处理非空响应
                        print(f"Received raw: {raw_data}")
                        
                        with self.lock:
                            self.response_received = True
                            self.last_response = raw_data
            except Exception as e:
                print(f"Error in receive thread: {e}")
                time.sleep(0.1)

    def close(self):
        self.running = False
        time.sleep(0.2)  # 给接收线程一点时间退出
        self.ser.close()
        print("Connection closed")



def test_connections():
    motor_ids = [0, 1, 6, 7]

    for i in motor_ids:
        port = f"/dev/ttyCH9344USB{i}"
        print(f"\nTesting motor on port {port}...")

        mc = MotorController(port=port)
        
        try:
            # mc.set_reduction_ratio(1)  # 设置减速比
            mc.motor_enable(True)  # optional if you need to enable before reading

            encoder_value = mc.query_encoder_angle()
            print(f"Motor {i} initial encoder value: {encoder_value}")

        except Exception as e:
            print(f"Motor {i} connection failed: {e}")

        finally:
            mc.motor_enable(False)
            mc.close()





if __name__ == "__main__":
    test_connections()
    # mc = MotorController(port='COM6')

    # i=7
    # port = f"/dev/ttyCH9344USB{i}"

    # mc = MotorController(port=port)

    # try:
    #     # 设置减速比
    #     mc.set_reduction_ratio(1)
        
    #     # 使能电机
    #     # mc.motor_enable(True)
        
    #     # 查询编码器角度
    #     encoder_value = mc.query_encoder_angle()
    #     print(f"Initial encoder value: {encoder_value}")
        
    #     # 转动电机
    #     # mc.set_angle(0)  # 转动6圈
        
    #     # 等待运动完成
    #     # time.sleep(2)
        
    #     # 再次查询编码器角度
    #     # encoder_value = mc.query_encoder_angle()
    #     # print(f"Final encoder value: {encoder_value}")
        
    #     # # 查询圈数
    #     # rotation_count = mc.query_rotation()
    #     # print(f"Rotation count: {rotation_count}")
        
    # finally:
    #     mc.motor_enable(False)
    #     mc.close()