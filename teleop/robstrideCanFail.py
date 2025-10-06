import can
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
    
    def __init__(self, bus: can.Bus, motor_id: int, mit_mode: bool = False, 
                 offset_func: Optional[Callable[[float], float]] = None):
        """
        初始化 RobStride 电机
        
        Args:
            bus: CAN 总线对象
            motor_id: 电机ID
            mit_mode: 是否使用 MIT 模式
            offset_func: 偏移量校正函数
        """
        self.bus = bus
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
    
    def calculate_can_id(self, comm_type, data2=0x0000, target_addr=None):
        """
        计算CAN扩展帧ID
        
        Args:
            comm_type: 通信类型 (5位)
            data2: 数据区2 (16位)
            target_addr: 目标地址 (8位)，默认为电机ID
            
        Returns:
            CAN扩展帧ID
        """
        if target_addr is None:
            target_addr = self.motor_id
            
        # 按照文档格式构建29位CAN ID
        # Bit28~24: 通信类型 (5位)
        # Bit23~8: 数据区2 (16位)  
        # Bit7~0: 目标地址 (8位)
        can_id = ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (target_addr & 0xFF)
        return can_id
    
    def initialize_motor(self):
        """初始化电机"""
        print(f"Initializing motor ID {self.motor_id}...")
        
        # 发送初始化命令1 - 使用正确的CAN ID格式
        init_data1 = bytes.fromhex("02 01 00")
        can_id1 = self.calculate_can_id(0x00, 0x07E8, self.motor_id)
        response1 = self.send_can_data(can_id1, init_data1)
        print(f"Sent initialization command 1 to ID {self.motor_id}")
        if response1:
            print(f"Response 1: {response1.hex(' ')}")
        time.sleep(0.1)
        
        # 发送初始化命令2 - 使用正确的CAN ID格式
        init_data2 = bytes.fromhex("08 00 c4 00 00 00 00 00 00")
        can_id2 = self.calculate_can_id(0x20, 0x07E8, self.motor_id)
        response2 = self.send_can_data(can_id2, init_data2)
        print(f"Sent initialization command 2 to ID {self.motor_id}")
        if response2:
            print(f"Response 2: {response2.hex(' ')}")
        time.sleep(0.1)
    
    def build_command(self, comm_type, data_bytes):
        """
        构建命令数据（不包含CAN ID）
        """
        # node_id = (self.motor_id << 2) + 0x08
        node_id = (self.motor_id << 3) | 0x04
        
        # 构造数据部分
        data_len = bytes([len(data_bytes)])
        frame_data = data_len + data_bytes
        return frame_data
    
    def float_to_hex(self, f):
        """浮点数转小端十六进制 - 与原始代码完全相同"""
        return struct.pack('<f', f)
    
    def hex_to_float(self, hex_data):
        """十六进制转浮点数"""
        return struct.unpack('<f', hex_data)[0]
    
    def send_can_data(self, can_id, data, expect_response=True, timeout=0.2):
        """
        通过CAN总线发送数据
        
        Args:
            can_id: CAN扩展帧ID
            data: 要发送的数据字节
            expect_response: 是否期望响应
            timeout: 超时时间
            
        Returns:
            响应数据或None
        """
        try:
            # 确保数据长度不超过8字节
            if len(data) > 8:
                data = data[:8]
            elif len(data) < 8:
                # 填充到8字节
                data = data + b'\x00' * (8 - len(data))
                
            # 创建CAN消息，使用扩展帧
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=True  # 使用扩展帧格式
            )
            self.bus.send(msg)
            print(f"Sent CAN frame: ID=0x{can_id:08X}, Data={data.hex(' ')}")
            
            # 如果期望响应，等待并返回响应
            if expect_response:
                return self.receive_response(timeout)
            else:
                return None
                
        except Exception as e:
            print(f"Error sending CAN frame: {e}")
            return None
    
    def send_command(self, comm_type, data_bytes, data2=0x0000, expect_response=True):
        """
        发送命令
        
        Args:
            comm_type: 通信类型
            data_bytes: 数据字节
            data2: 数据区2内容
            expect_response: 是否期望响应
            
        Returns:
            响应数据
        """
        # 构建CAN ID
        can_id = self.calculate_can_id(comm_type, data2, self.motor_id)
        
        # 构建数据
        frame_data = self.build_command(comm_type, data_bytes)
        
        # 发送CAN帧
        response = self.send_can_data(can_id, frame_data, expect_response)
        
        if response is not None:
            hex_data = frame_data.hex(' ')
            print(f"Sent command to motor ID {self.motor_id}: comm_type=0x{comm_type:02X}, data={hex_data}")
            print(f"Response: {response.hex(' ') if response else 'No response'}")
        else:
            print(f"No response from motor ID {self.motor_id}")
            
        return response
    
    def receive_response(self, timeout=0.2):
        """
        接收CAN响应
        
        Args:
            timeout: 超时时间
            
        Returns:
            响应数据或None
        """
        try:
            end_time = time.time() + timeout
            while time.time() < end_time:
                msg = self.bus.recv(timeout=0.01)
                if msg:
                    # 检查是否是发给我们的消息
                    target_addr = msg.arbitration_id & 0xFF
                    if target_addr == self.motor_id:
                        hex_data = msg.data.hex(' ')
                        print(f"Received from motor {self.motor_id}: ID=0x{msg.arbitration_id:08X}, Data={hex_data}")
                        return msg.data
            return None
        except Exception as e:
            print(f"Error receiving response: {e}")
            return None
    
    def set_position_mode(self):
        """设置位置模式"""
        print(f"Setting position mode for motor {self.motor_id}")
        data = bytes.fromhex("05 70 00 00 01 00 00 00")
        response = self.send_command(0x12, data)  # 0x12 = 18 设置参数
        time.sleep(0.2)
        return response
    
    def enable_motor(self):
        """使能电机"""
        print(f"Enabling motor {self.motor_id}")
        data = bytes.fromhex("00 00 00 00 00 00 00 00")
        response = self.send_command(0x03, data)  # 0x03 = 3 电机使能
        time.sleep(0.5)
        return response
    
    def get_parameter(self, parameter_index):
        """
        读取参数值
        
        Args:
            parameter_index: 参数索引
            
        Returns:
            参数值或None
        """
        print(f"Reading parameter 0x{parameter_index:04X} from motor {self.motor_id}")
        
        # 构建读取参数的命令数据
        # 参数索引需要以小端格式发送
        index_bytes = struct.pack('<H', parameter_index)
        data = index_bytes + b'\x00\x00\x00\x00\x00\x00'
        
        # 发送读取参数命令
        response = self.send_command(0x12, data, expect_response=True)  # 0x12 = 18 获取参数
        
        if response and len(response) >= 4:
            try:
                # 解析响应，假设参数值在响应的前4个字节（浮点数）
                value = self.hex_to_float(response[:4])
                print(f"Parameter 0x{parameter_index:04X} value: {value}")
                return value
            except Exception as e:
                print(f"Error parsing parameter value: {e}")
                return None
        else:
            print("Invalid response for parameter read")
            return None
    
    def get_vbus(self):
        """读取VBUS电压"""
        return self.get_parameter(0x701C)  # VBUS参数索引
    
    def get_position(self):
        """读取位置"""
        return self.get_parameter(0x7019)  # mechPos参数索引
    
    def get_velocity(self):
        """读取速度"""
        return self.get_parameter(0x701B)  # mechVel参数索引
    
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
        response1 = self.send_command(0x12, vel_data)  # 0x12 = 18 设置参数
        time.sleep(0.05)
        
        # 设置加速度
        acc_data = bytes.fromhex("25 70 00 00") + self.float_to_hex(acceleration)
        response2 = self.send_command(0x12, acc_data)  # 0x12 = 18 设置参数
        time.sleep(0.05)
        
        return response1, response2
    
    def set_position(self, position_rad):
        """
        设置位置
        
        Args:
            position_rad: 目标位置（弧度）
        """
        print(f"Setting motor {self.motor_id} position to {position_rad} rad")
        
        # 设置位置
        pos_data = bytes.fromhex("16 70 00 00") + self.float_to_hex(position_rad)
        response = self.send_command(0x12, pos_data)  # 0x12 = 18 设置参数
        time.sleep(0.1)
        return response
    
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
        """禁用电机"""
        print(f"Disabling motor {self.motor_id}")
        data = bytes.fromhex("00 00 00 00 00 00 00 00")
        response = self.send_command(0x04, data)  # 0x04 = 4 电机停止
        time.sleep(0.1)
        return response


class RobStrideController:
    """RobStride 控制器类，用于管理多个电机"""
    
    def __init__(self, channel: str = 'can0', interface: str = 'socketcan', bitrate: int = 1000000):
        """
        初始化控制器
        
        Args:
            channel: CAN通道 (例如 'can0')
            interface: CAN接口类型 (例如 'socketcan')
            bitrate: CAN总线比特率 (默认1Mbps)
        """
        self.bus = can.interface.Bus(interface=interface, channel=channel, bitrate=bitrate)
        
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
        motor = RobStrideMotor(self.bus, motor_id, mit_mode, offset_func)
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
    
    def read_all_parameters(self):
        """读取所有电机的参数"""
        for motor_id, motor in self.motors.items():
            print(f"\n--- Motor {motor_id} Parameters ---")
            vbus = motor.get_vbus()
            position = motor.get_position()
            velocity = motor.get_velocity()
            
            print(f"VBUS: {vbus} V" if vbus is not None else "VBUS: Failed to read")
            print(f"Position: {position} rad" if position is not None else "Position: Failed to read")
            print(f"Velocity: {velocity} rad/s" if velocity is not None else "Velocity: Failed to read")
    
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
            try:
                # 接收CAN消息
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    # 提取目标地址
                    target_addr = msg.arbitration_id & 0xFF
                    
                    if target_addr in self.motors:
                        motor = self.motors[target_addr]
                        hex_data = msg.data.hex(' ')
                        print(f"Background received from motor {target_addr}: ID=0x{msg.arbitration_id:08X}, Data={hex_data}")
            except Exception as e:
                print(f"Error reading CAN bus: {e}")
            
            time.sleep(0.001)
    
    def close(self):
        """关闭控制器"""
        self.stop_reading()
        self.bus.shutdown()


# 示例用法
def main():

    # sudo ip link set down can0
    # sudo ip link set can0 type can bitrate 1000000 loopback off
    # sudo ip link set up can0

    print("Starting multi-motor control with synchronization...")

    # 初始化控制器，使用1Mbps波特率
    controller = RobStrideController(channel='can0', interface='socketcan', bitrate=1000000)
    
    # 添加电机
    motor3 = controller.add_motor(motor_id=0x06)  # motor3

    try:
        # 设置所有电机为位置模式
        for motor in controller.motors.values():
            motor.set_position_mode()
        
        # 使能所有电机
        for motor in controller.motors.values():
            motor.enable_motor()
        
        time.sleep(2)

        # 读取电机参数
        print("\n--- Reading motor parameters before movement ---")
        controller.read_all_parameters()
        
        time.sleep(1)

        # 尝试移动到零位
        print("\n--- Moving to zero position ---")
        positions = {0x03: 0.0}
        controller.set_all_positions(positions)
        
        time.sleep(2)
        
        # 再次读取参数
        print("\n--- Reading motor parameters after movement ---")
        controller.read_all_parameters()

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