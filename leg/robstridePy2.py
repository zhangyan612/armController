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
    
    def __init__(self, ser: serial.Serial, motor_id: int, motor_type: str, mit_mode: bool = False, 
                 offset_func: Optional[Callable[[float], float]] = None):
        """
        初始化 RobStride 电机
        
        Args:
            ser: 串口对象
            motor_id: 电机ID
            motor_type: 电机类型 ('left' 或 'right')
            mit_mode: 是否使用 MIT 模式
            offset_func: 偏移量校正函数
        """
        self.ser = ser
        self.motor_id = motor_id
        self.motor_type = motor_type
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
        print(f"Initializing {self.motor_type} motor ID {self.motor_id}...")

        # 初始化命令1
        # Example: 41540007e80201000d0a (假设协议)
        init_cmd1 = self.build_can_frame(0x00, bytes.fromhex("02 01 00"), fixed_len=3)
        self.ser.write(init_cmd1)
        print(f"Sent to ID {self.motor_id}: {init_cmd1.hex(' ')}")
        time.sleep(0.1)

        # 初始化命令2
        # Example: 41542007e80800c400000000000d0a (假设协议)
        init_cmd2 = self.build_can_frame(0x20, bytes.fromhex("08 00 c4 00 00 00 00 00 00"), fixed_len=9)
        self.ser.write(init_cmd2)
        print(f"Sent to ID {self.motor_id}: {init_cmd2.hex(' ')}")
        time.sleep(0.1)

    
    def build_can_frame(self, command_type: int, data: bytes, fixed_len: int) -> bytes:
        """
        Build CAN frame with correct fixed data length.
        """
        ext_id = bytes([command_type, 0x07, 0xE8, self.CAN_ID])
        data_len = bytes([fixed_len])
        frame = b"\x41\x54" + ext_id + data_len + data + b"\x0d\x0a"
        return frame

    
    def float_to_hex(self, f):
        """浮点数转小端十六进制"""
        return struct.pack('<f', f)
    
    def hex_to_float(self, hex_str):
        """十六进制转浮点数"""
        return struct.unpack('<f', bytes.fromhex(hex_str))[0]
    
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
        print(f"Sent to {self.motor_type} motor ID {self.motor_id}: {hex_frame}")
        time.sleep(0.05)
        response = self.ser.read_all()
        if response:
            hex_response = response.hex()
            print(f"Received: {hex_response}")
        return response
    
    def enable_motor(self):
        print(f"Enabling {self.motor_type} motor {self.motor_id} in position mode")
        # First set position mode
        self.set_position_mode()
        time.sleep(0.1)
        # Then enable motor
        data = bytes.fromhex("14 08 00 00 00 00 00 00 00")
        frame = self.build_can_frame(0x18, data, fixed_len=0x0e)
        self.send_command(frame)
        time.sleep(0.5)
    
    def disable_motor(self):
        print(f"Disabling {self.motor_type} motor {self.motor_id}")
        data = bytes.fromhex("14 08 00 00 00 00 00 00 00")
        frame = self.build_can_frame(0x20, data, fixed_len=0x0e)
        self.send_command(frame)
    
   
    def set_position_mode(self):
        print(f"Setting position mode for {self.motor_type} motor {self.motor_id}")
        data = bytes.fromhex("14 08 05 70 00 00 05 00 00 00")
        frame = self.build_can_frame(0x90, data, fixed_len=0x0c)
        self.send_command(frame)
        time.sleep(0.2)


    
    def set_velocity_limit(self, velocity: float):
        print(f"Setting velocity limit for {self.motor_type} motor {self.motor_id} to {velocity}")
        vel_hex = self.float_to_hex(velocity)
        # Expected: 41549007e81408177000000000803f0d0a
        data = bytes.fromhex("14 08 17 70 00 00 00") + vel_hex
        frame = self.build_can_frame(0x90, data, 0x0b)
        self.send_command(frame)
            # 打印设置速度限制的信息，包括电机类型和ID
        time.sleep(0.1)
    
    def set_position(self, position: float):
        print(f"Setting position for {self.motor_type} motor {self.motor_id} to {position}")
        pos_hex = self.float_to_hex(position)
        # Expected: 41549007e8140816700000000000400d0a
        data = bytes.fromhex("14 08 16 70 00 00 00") + pos_hex
        frame = self.build_can_frame(0x90, data, fixed_len=0x0b)
        self.send_command(frame)
        time.sleep(0.1)
        
    def set_zero_position(self):
        print(f"Setting zero position for {self.motor_type} motor {self.motor_id}")
        # Expected: 41549007e8140816700000000000000d0a
        data = bytes.fromhex("14 08 16 70 00 00 00 00 00 00 00")
        frame = self.build_can_frame(0x90, data, 0x0b)
        self.send_command(frame)
        time.sleep(0.1)


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
    
    def add_motor(self, motor_id: int, motor_type: str, mit_mode: bool = False, 
                  offset_func: Optional[Callable[[float], float]] = None) -> RobStrideMotor:
        """
        添加电机
        
        Args:
            motor_id: 电机ID
            motor_type: 电机类型 ('left' 或 'right')
            mit_mode: 是否使用 MIT 模式
            offset_func: 偏移量校正函数
            
        Returns:
            电机对象
        """
        motor = RobStrideMotor(self.ser, motor_id, motor_type, mit_mode, offset_func)
        self.motors[motor_id] = motor
        return motor
    
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
                            # 这里需要实现解析方法
                            pass
                except Exception as e:
                    print(f"Error parsing data: {e}")
            
            time.sleep(0.001)
    
    def close(self):
        """关闭控制器"""
        self.stop_reading()
        self.ser.close()


# 示例用法 - 使用位置控制模式
def main():
    print("Starting motor control with position mode...")
    
    # 初始化串口
    ser = serial.Serial(
        port='COM21',  # 请根据实际情况修改端口
        baudrate=921600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.5
    )
    
    print("Initializing adapter...")
    # 发送初始化命令
    ser.write(bytes.fromhex("41 54 2b 41 54 0d 0a"))
    time.sleep(0.2)
    
    # 创建电机控制器 - 使用CAN ID 2
    motor = RobStrideMotor(ser, motor_id=0x02, motor_type='test')
    
    try:
        # 使能电机（会自动设置为位置模式）
        motor.enable_motor()
        time.sleep(1)
        
        # 设置速度限制为1.0
        motor.set_velocity_limit(1.0)
        time.sleep(0.5)
        
        # 移动到位置2.0
        motor.set_position(2.0)
        time.sleep(2)
        
        # 移动到位置0.0
        motor.set_position(0.0)
        time.sleep(2)
        
        # 移动到位置1.0
        motor.set_position(1.0)
        time.sleep(2)
        
        # 设置零点位置
        motor.set_zero_position()
        time.sleep(1)
        
        # 禁用电机
        motor.disable_motor()
        
        print("Position mode motor control completed successfully.")
    
    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        ser.close()
        print("Serial connection closed")


if __name__ == "__main__":
    main()


# 代码发送的指令和实际指令还是有所不同

# Setting position mode for test motor 2
# Sent to test motor ID 2: 41 54 90 07 e8 02 0a 0c 08 05 70 00 00 05 00 00 00 0d 0a
# Received: 4f4b0d0a

# 实际指令
# 41549007e8140805700000050000000d0a


# Sent to test motor ID 2: 41 54 18 07 e8 02 08 08 00 00 00 00 00 00 00 0d 0a

# 实际指令 运行 enable motor
# 41541807e8140800000000000000000d0a


# Setting velocity limit for test motor 2 to 1.0
# Sent to test motor ID 2: 41 54 90 07 e8 02 0b 14 08 17 70 00 00 00 00 00 80 3f 0d 0a

# 实际指令  41549007e81408177000000000803f0d0a



# Setting position for test motor 2 to 2.0
# Sent to test motor ID 2: 41 54 90 07 e8 02 0b 14 08 16 70 00 00 00 00 00 00 40 0d 0a

# 实际指令 41549007e8140816700000000000400d0a

# Setting position for test motor 2 to 0.0
# Sent to test motor ID 2: 41 54 90 07 e8 02 0b 14 08 16 70 00 00 00 00 00 00 00 0d 0a

# 实际指令 41549007e8140816700000000000000d0a

# Disabling test motor 2
# Sent to test motor ID 2: 41 54 20 07 e8 02 08 08 00 00 00 00 00 00 00 0d 0a

# 实际指令 41542007e8140800000000000000000d0a