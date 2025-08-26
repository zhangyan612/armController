import can
import struct
import time

# CAN总线配置
CAN_INTERFACE = 'can0'  # 根据实际接口修改
CAN_BITRATE = 1000000   # 1Mbps

# 通信类型定义
COMM_ENABLE = 0x3       # 电机使能
COMM_DISABLE = 0x4      # 电机停止
COMM_CTRL_MODE = 0x1    # 运控模式指令
COMM_WRITE_PARAM = 0x12 # 参数写入

# 参数索引定义 (参考文档第5-6页)
PARAM_RUN_MODE = 0x7005
PARAM_LOC_REF = 0x7016
PARAM_SPD_REF = 0x700A
PARAM_LIMIT_SPD = 0x7017

# 运行模式定义
MODE_PP = 1     # 位置模式(PP)
MODE_CSP = 5    # 位置模式(CSP)
MODE_CURRENT = 3 # 电流模式
MODE_SPEED = 2   # 速度模式

class RS04MotorController:
    def __init__(self, motor_id, master_id):
        self.motor_id = motor_id
        self.master_id = master_id
        self.bus = can.interface.Bus(
            channel=CAN_INTERFACE,
            bustype='socketcan',
            bitrate=CAN_BITRATE
        )
    
    def _build_frame(self, comm_type, data_bytes):
        """构建CAN扩展帧"""
        # 构造29位ID: [通信类型(5位) | 数据区2(16位) | 目标地址(8位)]
        frame_id = (comm_type << 24) | (self.master_id << 8) | self.motor_id
        return can.Message(
            arbitration_id=frame_id,
            data=data_bytes,
            is_extended_id=True
        )
    
    def enable_motor(self):
        """使能电机(通信类型3)"""
        frame = self._build_frame(COMM_ENABLE, bytes(8))  # 数据区全0
        self.bus.send(frame)
        print(f"Motor {self.motor_id} enabled")
    
    def disable_motor(self):
        """停止电机(通信类型4)"""
        frame = self._build_frame(COMM_DISABLE, bytes(8))  # 数据区全0
        self.bus.send(frame)
        print(f"Motor {self.motor_id} disabled")
    
    def set_run_mode(self, mode):
        """设置运行模式(通信类型18)"""
        # 构建数据: [Index(2字节) | 保留(2字节) | 值(4字节)]
        data = struct.pack('<Hxxf', PARAM_RUN_MODE, float(mode))
        frame = self._build_frame(COMM_WRITE_PARAM, data)
        self.bus.send(frame)
        print(f"Set mode: {mode}")

    def set_position(self, position_rad):
        """设置目标位置(通信类型18)"""
        data = struct.pack('<Hxxf', PARAM_LOC_REF, position_rad)
        frame = self._build_frame(COMM_WRITE_PARAM, data)
        self.bus.send(frame)
        print(f"Set position: {position_rad:.2f} rad")

    def set_velocity(self, velocity_rad_s):
        """设置目标速度(通信类型18)"""
        data = struct.pack('<Hxxf', PARAM_SPD_REF, velocity_rad_s)
        frame = self._build_frame(COMM_WRITE_PARAM, data)
        self.bus.send(frame)
        print(f"Set velocity: {velocity_rad_s:.2f} rad/s")

    def set_max_speed(self, max_speed_rad_s):
        """设置最大速度限制(通信类型18)"""
        data = struct.pack('<Hxxf', PARAM_LIMIT_SPD, max_speed_rad_s)
        frame = self._build_frame(COMM_WRITE_PARAM, data)
        self.bus.send(frame)
        print(f"Set max speed: {max_speed_rad_s:.2f} rad/s")

    def control_mode_command(self, torque, position, velocity, kp, kd):
        """运控模式指令(通信类型1)"""
        # 将浮点数转换为整型(参考文档转换方法)
        def float_to_uint(x, x_min, x_max, bits):
            span = x_max - x_min
            offset = x_min
            x = max(min(x, x_max), x_min)
            return int(((x - offset) * ((1 << bits) - 1)) / span)
        
        # 构建数据字节
        data = bytearray(8)
        # 转矩值转换(范围参考文档)
        torque_int = float_to_uint(torque, -120.0, 120.0, 16)
        data[0:2] = torque_int.to_bytes(2, 'big')
        # 位置值转换
        position_int = float_to_uint(position, -12.57, 12.57, 16)
        data[2:4] = position_int.to_bytes(2, 'big')
        # 速度值转换
        velocity_int = float_to_uint(velocity, -33.0, 33.0, 16)
        data[4:6] = velocity_int.to_bytes(2, 'big')
        # Kp/Kd转换(范围需根据实际调整)
        kp_int = float_to_uint(kp, 0.0, 500.0, 16)
        kd_int = float_to_uint(kd, 0.0, 100.0, 16)
        data[6] = kp_int >> 8
        data[7] = kp_int & 0xFF
        
        frame = self._build_frame(COMM_CTRL_MODE, data)
        self.bus.send(frame)

    def position_control(self, target_position, max_speed=5.0):
        """位置控制示例(CSP模式)"""
        self.set_run_mode(MODE_CSP)       # 设置为CSP模式
        self.set_max_speed(max_speed)     # 设置最大速度
        self.enable_motor()               # 使能电机
        self.set_position(target_position) # 设置目标位置
        print(f"Moving to position: {target_position} rad")

    def cleanup(self):
        """清理资源"""
        self.bus.shutdown()

# 使用示例
if __name__ == "__main__":
    # 初始化控制器 (电机ID=1, 主机ID=0xFD)
    controller = RS04MotorController(motor_id=1, master_id=0xFD)
    
    try:
        # 位置控制示例
        controller.position_control(target_position=3.14, max_speed=2.0)
        
        # 等待运动完成(实际应用中需要根据反馈判断)
        time.sleep(2)
        
        # 运控模式示例
        # controller.set_run_mode(0)  # 切换为运控模式
        # controller.enable_motor()
        # print("Running control mode...")
        
        # # 发送运控指令 (持续5秒)
        # start_time = time.time()
        # while time.time() - start_time < 5:
        #     controller.control_mode_command(
        #         torque=5.0,         # 5Nm
        #         position=0.0,        # 目标位置
        #         velocity=0.0,        # 目标速度
        #         kp=80.0,             # 位置增益
        #         kd=1.0               # 速度增益
        #     )
        #     time.sleep(0.01)        # 10ms控制周期
    
    finally:
        # 停止并清理
        controller.disable_motor()
        controller.cleanup()
        print("Controller shutdown")