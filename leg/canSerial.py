import serial
import struct
import time
import binascii

# 串口配置 (根据您的设备调整)
SERIAL_PORT = 'COM21'  # Windows
# SERIAL_PORT = '/dev/ttyUSB0'  # Linux
BAUDRATE = 921600      # 常见波特率: 115200, 230400, 460800, 921600

# CAN帧格式定义
CAN_ID_LEN = 2         # 标准帧ID长度(11位)
# CAN_ID_LEN = 8        # 扩展帧ID长度(29位) - 根据您的设备选择

# 通信类型定义
COMM_ENABLE = 0x3
COMM_DISABLE = 0x4
COMM_CTRL_MODE = 0x1
COMM_WRITE_PARAM = 0x12

# 参数索引
PARAM_RUN_MODE = 0x7005
PARAM_LOC_REF = 0x7016
PARAM_SPD_REF = 0x700A
PARAM_LIMIT_SPD = 0x7017

# 运行模式
MODE_CSP = 5

class SerialCANAdapter:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.1
        )
        # 初始化适配器 (命令根据设备手册)
        self._send_command("ATZ\r")  # 复位
        time.sleep(1)
        self._send_command("ATS0=1000000\r")  # 设置CAN波特率1Mbps
        self._send_command("ATL1\r")  # 启用换行
        self._send_command("ATO\r")   # 打开CAN通道
    
    def _send_command(self, command):
        """发送AT命令到适配器"""
        self.ser.write(command.encode())
        time.sleep(0.1)
        return self.ser.read_all().decode()
    
    def send_frame(self, can_id, data):
        """发送CAN帧"""
        # 构建帧: <ID><数据长度><数据>
        frame = f"{can_id:0{CAN_ID_LEN}X}{len(data):X}"
        for byte in data:
            frame += f"{byte:02X}"
        frame += "\r"
        self.ser.write(frame.encode())
    
    def receive_frame(self, timeout=0.1):
        """接收CAN帧"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.ser.in_waiting:
                line = self.ser.readline().decode().strip()
                if len(line) > CAN_ID_LEN + 1:  # 确保有ID和数据长度
                    try:
                        can_id = int(line[:CAN_ID_LEN], 16)
                        data_len = int(line[CAN_ID_LEN], 16)
                        data = bytes.fromhex(line[CAN_ID_LEN+1:CAN_ID_LEN+1+data_len*2])
                        return can_id, data
                    except ValueError:
                        continue
        return None, None
    
    def close(self):
        """关闭连接"""
        self.ser.close()

class RS04MotorController:
    def __init__(self, motor_id, master_id, serial_adapter):
        self.motor_id = motor_id
        self.master_id = master_id
        self.adapter = serial_adapter
    
    def _build_can_id(self, comm_type):
        """构建CAN ID"""
        # 格式: [通信类型(5位)|数据区2(16位)|目标地址(8位)]
        return (comm_type << 24) | (self.master_id << 8) | self.motor_id
    
    def enable_motor(self):
        """使能电机"""
        can_id = self._build_can_id(COMM_ENABLE)
        self.adapter.send_frame(can_id, bytes(8))  # 空数据
        print(f"Motor {self.motor_id} enabled")
    
    def disable_motor(self):
        """停止电机"""
        can_id = self._build_can_id(COMM_DISABLE)
        self.adapter.send_frame(can_id, bytes(8))  # 空数据
        print(f"Motor {self.motor_id} disabled")
    
    def set_run_mode(self, mode):
        """设置运行模式"""
        # 数据结构: [索引(2字节)|保留(2字节)|值(4字节)]
        data = struct.pack('<Hxxf', PARAM_RUN_MODE, float(mode))
        can_id = self._build_can_id(COMM_WRITE_PARAM)
        self.adapter.send_frame(can_id, data)
        print(f"Set mode: {mode}")
    
    def set_position(self, position_rad):
        """设置目标位置"""
        data = struct.pack('<Hxxf', PARAM_LOC_REF, position_rad)
        can_id = self._build_can_id(COMM_WRITE_PARAM)
        self.adapter.send_frame(can_id, data)
        print(f"Set position: {position_rad:.2f} rad")
    
    def set_max_speed(self, max_speed_rad_s):
        """设置最大速度"""
        data = struct.pack('<Hxxf', PARAM_LIMIT_SPD, max_speed_rad_s)
        can_id = self._build_can_id(COMM_WRITE_PARAM)
        self.adapter.send_frame(can_id, data)
        print(f"Set max speed: {max_speed_rad_s:.2f} rad/s")
    
    def control_mode_command(self, torque, position, velocity, kp, kd):
        """运控模式指令"""
        # 浮点数转换函数
        def float_to_uint(x, x_min, x_max, bits):
            span = x_max - x_min
            offset = x_min
            x = max(min(x, x_max), x_min)
            return int(((x - offset) * ((1 << bits) - 1)) / span)
        
        # 构建数据字节
        data = bytearray(8)
        # 转矩值(-120Nm ~ 120Nm)
        torque_int = float_to_uint(torque, -120.0, 120.0, 16)
        data[0:2] = torque_int.to_bytes(2, 'big')
        # 位置值(-12.57rad ~ 12.57rad)
        position_int = float_to_uint(position, -12.57, 12.57, 16)
        data[2:4] = position_int.to_bytes(2, 'big')
        # 速度值(-33rad/s ~ 33rad/s)
        velocity_int = float_to_uint(velocity, -33.0, 33.0, 16)
        data[4:6] = velocity_int.to_bytes(2, 'big')
        # Kp/Kd值
        kp_int = float_to_uint(kp, 0.0, 500.0, 16)
        kd_int = float_to_uint(kd, 0.0, 100.0, 16)
        data[6] = kp_int >> 8
        data[7] = kp_int & 0xFF
        
        can_id = self._build_can_id(COMM_CTRL_MODE)
        self.adapter.send_frame(can_id, data)
    
    def position_control(self, target_position, max_speed=5.0):
        """位置控制(CSP模式)"""
        self.set_run_mode(MODE_CSP)       # CSP模式
        self.set_max_speed(max_speed)     # 设置最大速度
        self.enable_motor()               # 使能电机
        self.set_position(target_position) # 设置目标位置
        print(f"Moving to position: {target_position} rad")
    
    def monitor_feedback(self, duration=5):
        """监控电机反馈"""
        print("Monitoring feedback for 5 seconds...")
        start_time = time.time()
        while time.time() - start_time < duration:
            can_id, data = self.adapter.receive_frame()
            if can_id and data:
                # 解析反馈帧 (通信类型2)
                if (can_id >> 24) == 0x2:
                    # 解析位置、速度、转矩等
                    # 实际解析需要根据文档中的帧格式
                    print(f"Received feedback: ID={hex(can_id)}, Data={binascii.hexlify(data)}")
            time.sleep(0.01)

# 使用示例
if __name__ == "__main__":
    # 初始化串口CAN适配器
    adapter = SerialCANAdapter(port=SERIAL_PORT, baudrate=BAUDRATE)
    
    try:
        # 初始化电机控制器 (电机ID=1, 主机ID=0xFD)
        controller = RS04MotorController(
            motor_id=2, 
            master_id=0,
            serial_adapter=adapter
        )
        
        # 位置控制示例
        controller.position_control(target_position=3.14, max_speed=2.0)
        time.sleep(2)  # 等待运动
        
        # 监控反馈
        controller.monitor_feedback()
        
        # 返回原点
        controller.position_control(target_position=0.0)
        time.sleep(2)
        
    finally:
        # 确保电机停止
        controller.disable_motor()
        adapter.close()
        print("Connection closed")