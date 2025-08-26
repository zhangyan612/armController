import serial
import time
import binascii
import struct
from enum import Enum
from typing import Optional, Callable
import threading 

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
        
        # 发送初始化命令1 - 与原始代码完全相同
        init_cmd1 = bytes.fromhex(f"41 54 00 07 e8 {self.motor_id:02x} 02 01 00 0d 0a")
        self.ser.write(init_cmd1)
        print(f"Sent to ID {self.motor_id}: {init_cmd1.hex(' ')}")
        time.sleep(0.1)
        
        # 发送初始化命令2 - 与原始代码完全相同
        init_cmd2 = bytes.fromhex(f"41 54 20 07 e8 {self.motor_id:02x} 08 00 c4 00 00 00 00 00 00 0d 0a")
        self.ser.write(init_cmd2)
        print(f"Sent to ID {self.motor_id}: {init_cmd2.hex(' ')}")
        time.sleep(0.1)
    
    def build_command(self, comm_type, data_bytes):
        """
        构建 CAN 帧
        """
        # node_id = (self.motor_id << 2) + 0x08
        node_id = (self.motor_id << 3) | 0x04
        # 根据第二套代码固定的 CAN ID 规则，构造扩展 ID
        ext_id_bytes = bytes([comm_type, 0x07, 0xE8, node_id])

        data_len = bytes([len(data_bytes)])
        frame = b"\x41\x54" + ext_id_bytes + data_len + data_bytes + b"\x0d\x0a"
        return frame
    
    def float_to_hex(self, f):
        """浮点数转小端十六进制 - 与原始代码完全相同"""
        return struct.pack('<f', f)
    
    def send_command(self, frame):
        """
        发送帧并读取响应 - 与原始代码完全相同
        
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
        """设置位置模式 - 与原始代码完全相同"""
        print(f"Setting position mode for motor {self.motor_id}")
        data = bytes.fromhex("05 70 00 00 01 00 00 00")
        frame = self.build_command(0x90, data)
        self.send_command(frame)
        time.sleep(0.2)
    
    def enable_motor(self):
        """使能电机 - 与原始代码完全相同"""
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
        移动到指定位置（兼容旧方法）
        
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
        """禁用电机 - 与原始代码完全相同"""
        print(f"Disabling motor {self.motor_id}")
        data = bytes.fromhex("00 00 00 00 00 00 00 00")
        frame = self.build_command(0x20, data)
        self.send_command(frame)


class RobStrideController:
    """RobStride 控制器类，用于管理多个电机"""
    
    def __init__(self, port: str, baudrate: int = 921600):
        """
        初始化控制器
        
        Args:
            port: 串口端口
            baudrate: 波特率
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        
        self.motors = {}
        self.running = False
        self.read_thread = None
    
    def add_motor(self, motor_id: int, mit_mode: bool = False, 
                  offset_func: Optional[Callable[[float], float]] = None) -> RobStrideMotor:
        """
        添加电机
        
        Args:
            motor_id: 电机ID
            mit_mode: 是否使用 MIT 模式
            offset_func: 偏移量校正函数
            
        Returns:
            电机对象
        """
        motor = RobStrideMotor(self.ser, motor_id, mit_mode, offset_func)
        self.motors[motor_id] = motor
        return motor
    
    def set_all_velocity_acceleration(self, velocity=1.0, acceleration=5.0):
        """为所有电机设置相同的速度和加速度"""
        for motor in self.motors.values():
            motor.set_velocity_acceleration(velocity, acceleration)
    
    def set_all_positions(self, positions):
        """为所有电机设置位置（可分别设置不同位置）"""
        for motor_id, position in positions.items():
            if motor_id in self.motors:
                self.motors[motor_id].set_position(position)
    
    def move_all_to_positions(self, positions, velocity=1.0, acceleration=5.0):
        """
        同步移动所有电机到指定位置
        
        Args:
            positions: 电机ID到目标位置的映射
            velocity: 速度 (rad/s)
            acceleration: 加速度 (rad/s²)
        """
        print(f"Moving all motors to positions with velocity {velocity} rad/s and acceleration {acceleration} rad/s²")
        
        # 1. 设置所有电机的速度和加速度
        self.set_all_velocity_acceleration(velocity, acceleration)
        
        # 2. 设置所有电机的位置（几乎同时发送）
        self.set_all_positions(positions)
    
    def start_reading(self):
        """开始读取数据"""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
    
    def stop_reading(self):
        """停止读取数据"""
        self.running = False
        if self.read_thread:
            self.read_thread.join()
    
    def _read_loop(self):
        """读取循环"""
        while self.running:
            if self.ser.in_waiting:
                # 读取数据并解析
                data = self.ser.read_all()
                # 这里需要根据实际协议解析数据帧
                # 由于协议比较复杂，这里只提供框架
                try:
                    # 假设数据以 "AT" 开头，以 "\r\n" 结尾
                    if data.startswith(b"AT") and data.endswith(b"\r\n"):
                        # 提取扩展ID和数据
                        ext_id_bytes = data[2:6]
                        data_len = data[6]
                        payload = data[7:7+data_len]
                        
                        ext_id = int.from_bytes(ext_id_bytes, byteorder='big')
                        can_id = ext_id & 0xFF
                        
                        if can_id in self.motors:
                            self.motors[can_id].analysis(payload, ext_id)
                except Exception as e:
                    print(f"Error parsing data: {e}")
            
            time.sleep(0.001)
    
    def close(self):
        """关闭控制器"""
        self.stop_reading()
        self.ser.close()


# 示例用法
def main():
    print("Starting multi-motor control with synchronization...")

    # 初始化控制器
    controller = RobStrideController(port='COM21')
    
    # 初始化适配器
    controller.ser.write(bytes.fromhex("41 54 2b 41 54 0d 0a"))
    time.sleep(0.2)

    # 添加电机
    motor1 = controller.add_motor(motor_id=0x01)  # motor1
    motor2 = controller.add_motor(motor_id=0x02)  # motor2
    motor3 = controller.add_motor(motor_id=0x03)  # motor3
    motor4 = controller.add_motor(motor_id=0x04)  # motor4

    try:
        # 设置所有电机为位置模式
        for motor in controller.motors.values():
            motor.set_position_mode()
        
        # 使能所有电机
        for motor in controller.motors.values():
            motor.enable_motor()
        
        time.sleep(2)

        # 使用同步方法移动所有电机到初始位置
        initial_positions = {0x01: 0.0, 0x02: 0.0, 0x03: 0.0, 0x04: 0.0}
        controller.move_all_to_positions(initial_positions, velocity=2.0, acceleration=10.0)
        time.sleep(1)

        # 移动到不同位置
        positions_1 = {0x01: 3.0, 0x02: 3.0, 0x03: 3.0, 0x04: 3.0}
        controller.move_all_to_positions(positions_1, velocity=1.5, acceleration=8.0)
        time.sleep(1)

        # 再次移动回初始位置
        controller.move_all_to_positions(initial_positions, velocity=2.5, acceleration=5.0)
        time.sleep(1)

        # 禁用所有电机
        for motor in controller.motors.values():
            motor.disable_motor()

        print("Multi-motor control with synchronization completed successfully.")

    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        controller.close()
        print("Controller closed")


if __name__ == "__main__":
    main()