import serial
import serial.tools.list_ports
import time

class ServoController:
    def __init__(self, port='COM21', baudrate=9600):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.connect()
    
    def connect(self):
        """尝试连接到指定的串口"""
        try:
            # 关闭现有连接（如果存在）
            if self.ser and self.ser.is_open:
                self.ser.close()
                
            # 创建新的串口连接
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=2  # 增加超时时间
            )
            time.sleep(0.5)  # 延长等待时间确保初始化完成
            print(f"✅ 成功打开串口 {self.port}")
            return True
        except (serial.SerialException, OSError) as e:
            print(f"❌ 无法打开串口 {self.port}: {e}")
            self.list_available_ports()
            return False
    
    def list_available_ports(self):
        """列出所有可用串口"""
        print("🔍 扫描可用串口...")
        ports = serial.tools.list_ports.comports()
        if ports:
            print("📋 可用串口:")
            for port in ports:
                print(f"  - {port.device}: {port.description}")
        else:
            print("⚠️ 没有找到可用串口")
        return [port.device for port in ports]
    
    def move_servo(self, channel, position, duration):
        """控制单个舵机
        :param channel: 舵机通道 (1-24)
        :param position: 舵机位置 (500-2500)
        :param duration: 执行时间 (100-9999) 毫秒
        """
        if not 1 <= channel <= 24:
            raise ValueError("通道号必须在1-24之间")
        if not 500 <= position <= 2500:
            raise ValueError("舵机位置必须在500-2500之间")
        if not 100 <= duration <= 9999:
            raise ValueError("执行时间必须在100-9999之间")
        
        command = f"#{channel}P{position}T{duration}\r\n"
        self._send_command(command)
    
    def move_servos(self, servos, duration):
        """控制多个舵机
        :param servos: 舵机参数列表 [(channel, position), ...]
        :param duration: 执行时间 (100-9999) 毫秒
        """
        if not 100 <= duration <= 9999:
            raise ValueError("执行时间必须在100-9999之间")
        
        command = ""
        for channel, position in servos:
            if not 1 <= channel <= 24:
                raise ValueError("通道号必须在1-24之间")
            if not 500 <= position <= 2500:
                raise ValueError("舵机位置必须在500-2500之间")
            command += f"#{channel}P{position}"
        
        command += f"T{duration}\r\n"
        self._send_command(command)
    
    def run_action_group(self, group_id, cycles=1):
        """执行单个动作组
        :param group_id: 动作组编号
        :param cycles: 循环次数 (1-255)
        """
        if not 1 <= cycles <= 255:
            raise ValueError("循环次数必须在1-255之间")
        
        command = f"#{group_id}GC{cycles}\r\n"
        self._send_command(command)
    
    def run_action_groups(self, group_ids, cycles=1):
        """执行多个动作组
        :param group_ids: 动作组编号列表 [id1, id2, ...]
        :param cycles: 循环次数 (1-255)
        """
        if not 1 <= cycles <= 255:
            raise ValueError("循环次数必须在1-255之间")
        
        command = ""
        for group_id in group_ids:
            command += f"#{group_id}G"
        
        command += f"C{cycles}\r\n"
        self._send_command(command)
    
    def stop(self):
        """停止所有动作"""
        self._send_command("#STOP\r\n")
    
    def _send_command(self, command):
        """发送命令到串口"""
        if not self.ser or not self.ser.is_open:
            print("⚠️ 串口未连接，尝试重新连接...")
            if not self.connect():
                raise ConnectionError("无法连接到串口设备")
        
        try:
            print(f"📤 发送命令: {command.strip()}")
            self.ser.write(command.encode())
            time.sleep(0.1)  # 命令间短暂延迟
        except serial.SerialException as e:
            print(f"❌ 发送命令失败: {e}")
            # 尝试重新连接
            print("🔄 尝试重新连接串口...")
            self.connect()
            if self.ser and self.ser.is_open:
                print("✅ 重新连接成功，重试发送命令")
                self.ser.write(command.encode())
    
    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"🔌 已关闭串口 {self.port}")


# 使用示例
if __name__ == "__main__":
    # 尝试连接舵机控制器
    controller = ServoController(port='COM21')
    
    if controller.ser and controller.ser.is_open:
        try:
            print("\n===== 测试单个舵机控制 =====")
            controller.move_servo(channel=1, position=1500, duration=1000)  # 1号舵机到中位
            time.sleep(1)  # 等待舵机运动
            
            # print("\n===== 测试多个舵机控制 =====")
            # controller.move_servos(
            #     servos=[(1, 600), (2, 900), (8, 2500)],
            #     duration=2000
            # )
            # time.sleep(2)  # 等待舵机运动
            
            # print("\n===== 测试动作组执行 =====")
            # controller.run_action_group(group_id=1, cycles=2)  # 执行动作组1循环2次
            # time.sleep(1)
            
            # print("\n===== 测试多个动作组 =====")
            # controller.run_action_groups(
            #     group_ids=[1, 3, 1],
            #     cycles=1
            # )
            # time.sleep(1)
            
            # 可选：停止命令
            print("\n===== 测试停止命令 =====")
            controller.stop()
            
            print("\n✅ 所有测试命令已发送")
            
        except Exception as e:
            print(f"❌ 发生错误: {e}")
        finally:
            controller.close()
    else:
        print("❌ 无法连接舵机控制器，请检查连接")