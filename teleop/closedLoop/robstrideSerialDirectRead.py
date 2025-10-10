import serial
import time
import struct
import threading
from enum import Enum
from typing import Optional, Callable

# ==================== 电位器读取部分 ====================

def hex_to_percentage(hex_data):
    percentages = []
    # 每4个十六进制字符（2字节）解析为一个数值
    for i in range(0, len(hex_data), 4):
        high_byte = int(hex_data[i:i+2], 16)
        low_byte = int(hex_data[i+2:i+4], 16)
        value = (high_byte << 8) + low_byte
        percentages.append(value)
    return percentages

def read_serial_data(ser, expected_bytes):
    data = b''
    start_time = time.time()
    while len(data) < expected_bytes and (time.time() - start_time) < ser.timeout:
        if ser.in_waiting > 0:
            data += ser.read(ser.in_waiting)
    return data.hex()

def process_serial_commands(ser):
    commands = [0xEE]  # 只使用0xEE命令读取编码器位置
    
    try:
        for command in commands:
            ser.write(bytes([command]))
            time.sleep(0.01)  # 添加短暂延迟确保数据接收
            
            # 读取足够的数据：12路编码器 × 2字节/路 + 头尾标记
            # 假设头尾各1字节标记，所以总共需要26字节
            response = read_serial_data(ser, 26)
            
            if command == 0xEE and len(response) >= 26:
                # 去掉头尾标记（各1字节，即2个十六进制字符）
                clean_response = response[2:-2]
                percentages = hex_to_percentage(clean_response)
                return percentages
            else:
                print(f"Invalid response length: {len(response)}")
                return None
                
    except Exception as e:
        print(f"Serial communication error: {e}")
        return None

def restrict_range(value, lower=-2, upper=2):
    """Restricts a number within the range [-2, 2]."""
    return max(lower, min(value, upper))

def map_to_motor(value, in_min=0, in_max=9940, out_min=0, out_max=6.28):
    """Maps a value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# ==================== 电机控制部分 ====================

# 常量定义
P_MIN = -12.5
P_MAX = 12.5
V_MIN = -44.0
V_MAX = 44.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0
T_MIN = -17.0
T_MAX = 17.0

# 通信类型枚举
class CommunicationType(Enum):
    GET_ID = 0
    MOTION_CONTROL = 1
    MOTOR_ENABLE = 3
    MOTOR_STOP = 4
    SET_SINGLE_PARAMETER = 17
    GET_SINGLE_PARAMETER = 18
    SET_POS_ZERO = 6
    CAN_ID = 7
    MOTOR_DATA_SAVE = 22
    BAUD_RATE_CHANGE = 23
    PROACTIVE_ESCALATION_SET = 24
    MOTOR_MODE_SET = 25

# 电机模式枚举
class MotorMode(Enum):
    MOVE_CONTROL = 0
    POS_CONTROL = 1
    SPEED_CONTROL = 2
    ELECT_CONTROL = 3
    SET_ZERO = 4
    CSP_CONTROL = 5

# 参数索引列表
INDEX_LIST = [
    0x7005,  # run_mode
    0x7006,  # iq_ref
    0x700A,  # spd_ref
    0x700B,  # imit_torque
    0x7010,  # cur_kp
    0x7011,  # cur_ki
    0x7012,  # cur_filt_gain
    0x7016,  # loc_ref
    0x7017,  # limit_spd
    0x7018,  # limit_cur
    0x7019,  # mechPos
    0x701A,  # iqf
    0x701B,  # mechVel
    0x701C,  # VBUS
    0x701D,  # rotation
]

class DataReadWrite:
    """数据读写类，用于存储电机参数"""
    def __init__(self):
        self.run_mode = Parameter(INDEX_LIST[0])
        self.iq_ref = Parameter(INDEX_LIST[1])
        self.spd_ref = Parameter(INDEX_LIST[2])
        self.imit_torque = Parameter(INDEX_LIST[3])
        self.cur_kp = Parameter(INDEX_LIST[4])
        self.cur_ki = Parameter(INDEX_LIST[5])
        self.cur_filt_gain = Parameter(INDEX_LIST[6])
        self.loc_ref = Parameter(INDEX_LIST[7])
        self.limit_spd = Parameter(INDEX_LIST[8])
        self.limit_cur = Parameter(INDEX_LIST[9])
        self.mechPos = Parameter(INDEX_LIST[10])
        self.iqf = Parameter(INDEX_LIST[11])
        self.mechVel = Parameter(INDEX_LIST[12])
        self.VBUS = Parameter(INDEX_LIST[13])
        self.rotation = Parameter(INDEX_LIST[14])

class Parameter:
    """参数类，用于存储单个参数的值"""
    def __init__(self, index):
        self.index = index
        self.data = 0.0

class PosInfo:
    """位置信息类，用于存储电机的位置、速度、扭矩和温度信息"""
    def __init__(self):
        self.angle = 0.0
        self.speed = 0.0
        self.torque = 0.0
        self.temp = 0.0
        self.pattern = 0

class MotorSetAll:
    """电机设置类，用于存储所有电机设置"""
    def __init__(self):
        self.set_motor_mode = MotorMode.MOVE_CONTROL
        self.set_Torque = 0.0
        self.set_angle = 0.0
        self.set_speed = 0.0
        self.set_Kp = 0.0
        self.set_Kd = 0.0
        self.set_current = 0.0
        self.set_limit_speed = 0.0
        self.set_acceleration = 0.0
        self.set_limit_cur = 0.0

class RobStrideMotor:
    """RobStride 电机控制类"""
    
    def __init__(self, ser: serial.Serial, motor_id: int, mit_mode: bool = False, 
                 offset_func: Optional[Callable[[float], float]] = None):
        """
        初始化 RobStride 电机
        
        Args:
            ser: 串口对象
            motor_id: 电机ID
            mit_mode: 是否使用 MIT 模式
            offset_func: 偏移量校正函数
        """
        self.ser = ser
        self.motor_id = motor_id
        self.CAN_ID = motor_id
        self.Master_CAN_ID = 0xFD
        self.MIT_Mode = mit_mode
        self.MIT_Type = 0  # operationControl
        self.Motor_Offset_MotoFunc = offset_func
        self.error_code = 0
        self.Unique_ID = 0
        self.output = 0.0
        
        self.drw = DataReadWrite()
        self.Pos_Info = PosInfo()
        self.Motor_Set_All = MotorSetAll()
        
        # 初始化电机
        self.initialize_motor()
    
    def initialize_motor(self):
        """初始化电机"""
        print(f"Initializing motor ID {self.motor_id}...")
        
        # 发送初始化命令1
        init_cmd1 = bytes.fromhex(f"41 54 00 07 e8 {self.motor_id:02x} 02 01 00 0d 0a")
        self.ser.write(init_cmd1)
        print(f"Sent to ID {self.motor_id}: {init_cmd1.hex(' ')}")
        time.sleep(0.1)
        
        # 发送初始化命令2
        init_cmd2 = bytes.fromhex(f"41 54 20 07 e8 {self.motor_id:02x} 08 00 c4 00 00 00 00 00 00 0d 0a")
        self.ser.write(init_cmd2)
        print(f"Sent to ID {self.motor_id}: {init_cmd2.hex(' ')}")
        time.sleep(0.1)
    
    def build_command(self, comm_type, data_bytes):
        """
        构建 CAN 帧
        """
        node_id = (self.motor_id << 3) | 0x04
        ext_id_bytes = bytes([comm_type, 0x07, 0xE8, node_id])

        data_len = bytes([len(data_bytes)])
        frame = b"\x41\x54" + ext_id_bytes + data_len + data_bytes + b"\x0d\x0a"
        return frame
    
    def float_to_hex(self, f):
        """浮点数转小端十六进制"""
        return struct.pack('<f', f)
    
    def send_command(self, frame):
        """
        发送帧并读取响应
        
        Args:
            frame: 要发送的帧
            
        Returns:
            响应数据
        """
        self.ser.write(frame)
        hex_frame = frame.hex(' ')
        print(f"Sent to motor ID {self.motor_id}: {hex_frame}")
        time.sleep(0.05)
        response = self.ser.read_all()
        if response:
            hex_response = response.hex()
            print(f"Received: {hex_response}")
        return response
    
    def set_position_mode(self):
        """设置位置模式"""
        print(f"Setting position mode for motor {self.motor_id}")
        data = bytes.fromhex("05 70 00 00 01 00 00 00")
        frame = self.build_command(0x90, data)
        self.send_command(frame)
        time.sleep(0.2)
    
    def enable_motor(self):
        """使能电机"""
        print(f"Enabling motor {self.motor_id}")
        data = bytes.fromhex("00 00 00 00 00 00 00 00")
        frame = self.build_command(0x18, data)
        self.send_command(frame)
        time.sleep(0.5)
    
    def set_velocity_acceleration(self, velocity=1.0, acceleration=5.0):
        """
        设置速度和加速度
        
        Args:
            velocity: 速度 (rad/s)
            acceleration: 加速度 (rad/s²)
        """
        print(f"Setting motor {self.motor_id} velocity to {velocity} rad/s and acceleration to {acceleration} rad/s²")
        
        # 设置速度
        vel_data = bytes.fromhex("24 70 00 00") + self.float_to_hex(velocity)
        vel_frame = self.build_command(0x90, vel_data)
        self.send_command(vel_frame)
        time.sleep(0.05)
        
        # 设置加速度
        acc_data = bytes.fromhex("25 70 00 00") + self.float_to_hex(acceleration)
        acc_frame = self.build_command(0x90, acc_data)
        self.send_command(acc_frame)
        time.sleep(0.05)
    
    def set_position(self, position_rad):
        """
        设置位置
        
        Args:
            position_rad: 目标位置（弧度）
        """
        print(f"Setting motor {self.motor_id} position to {position_rad} rad")
        
        # 设置位置
        pos_data = bytes.fromhex("16 70 00 00") + self.float_to_hex(position_rad)
        pos_frame = self.build_command(0x90, pos_data)
        self.send_command(pos_frame)
        time.sleep(0.1)
    
    def move_to_position(self, position_rad, velocity=1.0, acceleration=5.0):
        """
        移动到指定位置
        
        Args:
            position_rad: 目标位置（弧度）
            velocity: 速度 (rad/s)
            acceleration: 加速度 (rad/s²)
        """
        print(f"Moving motor {self.motor_id} to {position_rad} rad with velocity {velocity} rad/s and acceleration {acceleration} rad/s²")
        
        # 设置速度和加速度
        self.set_velocity_acceleration(velocity, acceleration)
        
        # 设置位置
        self.set_position(position_rad)
    
    def disable_motor(self):
        """禁用电机"""
        print(f"Disabling motor {self.motor_id}")
        data = bytes.fromhex("00 00 00 00 00 00 00 00")
        frame = self.build_command(0x20, data)
        self.send_command(frame)

# ==================== 主控制程序 ====================

class PotentiometerMotorController:
    """电位器控制电机的主控制器类"""
    
    def __init__(self, pot_port='COM9', motor_port='COM21', baudrate=921600):
        """
        初始化控制器
        
        Args:
            pot_port: 电位器串口
            motor_port: 电机控制串口
            baudrate: 电机控制串口波特率
        """
        # 初始化电位器串口
        self.pot_ser = serial.Serial(
            port=pot_port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        # 初始化电机控制串口
        self.motor_ser = serial.Serial(
            port=motor_port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        
        # 初始化适配器
        self.motor_ser.write(bytes.fromhex("41 54 2b 41 54 0d 0a"))
        time.sleep(0.2)
        
        # 存储电机对象
        self.motors = {}
        
        # 电位器到电机的映射关系 (电位器索引: 电机ID)
        self.pot_motor_mapping = {}
        
        # 控制循环运行标志
        self.running = False
        self.control_thread = None
    
    def add_motor(self, motor_id):
        """添加电机"""
        motor = RobStrideMotor(self.motor_ser, motor_id)
        self.motors[motor_id] = motor
        return motor
    
    def set_pot_motor_mapping(self, pot_index, motor_id):
        """设置电位器到电机的映射关系"""
        self.pot_motor_mapping[pot_index] = motor_id
        print(f"Mapped potentiometer {pot_index} to motor {motor_id}")
    
    def setup_motors(self):
        """设置所有电机为位置模式并启用"""
        for motor_id, motor in self.motors.items():
            motor.set_position_mode()
            motor.enable_motor()
        
        time.sleep(2)
        print("All motors set up and enabled")
    
    def read_potentiometers(self):
        """读取电位器值"""
        return process_serial_commands(self.pot_ser)
    
    def control_loop(self, interval=0.1):
        """控制循环 - 读取电位器并控制电机"""
        try:
            while self.running:
                # 读取电位器值
                pot_values = self.read_potentiometers()
                
                if pot_values and len(pot_values) >= 12:
                    # 处理每个映射的电位器-电机对
                    for pot_index, motor_id in self.pot_motor_mapping.items():
                        if pot_index < len(pot_values) and motor_id in self.motors:
                            # 获取电位器值并映射到电机范围
                            pot_value = pot_values[pot_index]
                            mapped_value = map_to_motor(pot_value)
                            mapped_value = restrict_range(mapped_value, 0, 6.28)
                            
                            # 控制电机
                            self.motors[motor_id].set_position(mapped_value)
                            
                            print(f"Pot {pot_index} -> Motor {motor_id}: {pot_value} -> {mapped_value:.2f} rad")
                        else:
                            print(f"Invalid mapping: Pot {pot_index} -> Motor {motor_id}")
                else:
                    print("Failed to read potentiometer values")
                
                time.sleep(interval)
                
        except Exception as e:
            print(f"Control loop error: {e}")
    
    def start(self):
        """启动控制循环"""
        if not self.running:
            self.running = True
            self.control_thread = threading.Thread(target=self.control_loop)
            self.control_thread.daemon = True
            self.control_thread.start()
            print("Control loop started")
    
    def stop(self):
        """停止控制循环并禁用所有电机"""
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        
        # 禁用所有电机
        for motor in self.motors.values():
            motor.disable_motor()
        
        # 关闭串口
        self.pot_ser.close()
        self.motor_ser.close()
        
        print("Controller stopped and motors disabled")

# ==================== 主程序 ====================

def main():
    """主函数"""
    print("Starting potentiometer-controlled motor system...")
    
    # 创建控制器
    controller = PotentiometerMotorController(pot_port='COM9', motor_port='COM21')
    
    try:
        # 添加电机 (例如添加ID为3的电机)
        controller.add_motor(0x03)
        
        # 设置电位器到电机的映射 (例如电位器6控制电机3)
        controller.set_pot_motor_mapping(5, 0x03)  # 注意: 电位器索引从0开始，所以6号电位器是索引5
        
        # 设置电机
        controller.setup_motors()
        
        # 启动控制循环
        controller.start()
        
        # 保持程序运行
        print("System running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nStopping system...")
    finally:
        controller.stop()
        print("System stopped.")

if __name__ == "__main__":
    main()