import serial
import time
import binascii
import struct
from enum import Enum
from typing import Optional, Callable, List, Iterable
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

# 通信类型枚举（保持不变）
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

# 电机模式枚举（保持不变）
class MotorMode(Enum):
    MOVE_CONTROL = 0
    POS_CONTROL = 1
    SPEED_CONTROL = 2
    ELECT_CONTROL = 3
    SET_ZERO = 4
    CSP_CONTROL = 5

# 参数索引列表（保持不变）
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

class Parameter:
    def __init__(self, index):
        self.index = index
        self.data = 0.0

class DataReadWrite:
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

class PosInfo:
    def __init__(self):
        self.angle = 0.0
        self.speed = 0.0
        self.torque = 0.0
        self.temp = 0.0
        self.pattern = 0

class MotorSetAll:
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
    """
    RobStride 电机控制类：兼容 1~4 号电机；支持 速度模式(速度/加速度可调) 与 位置模式。
    node_id 计算默认遵循你“能工作的公式”：
      - left:  node_id = motor_id
      - right: node_id = (motor_id << 2) + 0x08
    如果你想切换到“文档规范”(node_id = motor_id << 2)，把 use_side_mapping=False 即可。
    """

    def __init__(
        self,
        ser: serial.Serial,
        motor_id: int,
        motor_type: str = "right",           # 'left' 或 'right'
        mit_mode: bool = False,
        offset_func: Optional[Callable[[float], float]] = None,
        use_side_mapping: bool = True        # 默认按“只有这样才能工作”的方式
    ):
        self.ser = ser
        self.motor_id = motor_id            # 你的电机 CAN ID: 1,2,3,4
        self.motor_type = motor_type
        self.use_side_mapping = use_side_mapping
        self.CAN_ID = motor_id
        self.Master_CAN_ID = 0xFD
        self.MIT_Mode = mit_mode
        self.MIT_Type = 0
        self.Motor_Offset_MotoFunc = offset_func
        self.error_code = 0
        self.Unique_ID = 0
        self.output = 0.0

        self.drw = DataReadWrite()
        self.Pos_Info = PosInfo()
        self.Motor_Set_All = MotorSetAll()

        self.initialize_motor()

    # === 工具函数 ===
    def _float_tail3(self, value: float) -> bytes:
        """
        与“第二套代码”一致：float32小端后取最后3字节。
        例如 2.0 -> b'\x00\x00@'，5.0 -> b'\x00\xa0@'
        """
        b = struct.pack('<f', value)
        return b[1:]  # 取索引 1,2,3

    def _compute_node_id(self, motor_id: Optional[int] = None) -> int:
        """
        计算 node_id：
          - 侧别映射（默认）：left 用 motor_id，right 用 (motor_id<<2)+0x08
          - 文档规范（可选）：node_id = motor_id << 2
        """
        mid = self.motor_id if motor_id is None else motor_id
        if self.use_side_mapping:
            if self.motor_type.lower() == 'left':
                return mid & 0xFF
            else:
                return ((mid << 2) + 0x08) & 0xFF
        else:
            return ((mid << 2) & 0xFF)

    def build_command(self, comm_type: int, data_bytes: bytes, motor_id: Optional[int] = None) -> bytes:
        node_id = self._compute_node_id(motor_id)
        ext_id_bytes = bytes([comm_type & 0xFF, 0x07, 0xE8, node_id])
        data_len = bytes([len(data_bytes) & 0xFF])
        frame = b"\x41\x54" + ext_id_bytes + data_len + data_bytes + b"\x0D\x0A"
        return frame

    def send_command(self, frame: bytes):
        self.ser.write(frame)
        print(f"Sent: {frame.hex(' ').upper()}")
        time.sleep(0.05)
        resp = self.ser.read_all()
        if resp:
            print(f"Received: {resp.hex().upper()}")
        return resp

    # === 初始化/模式/使能/失能 ===
    def initialize_motor(self):
        """
        与你原始逻辑一致：按 motor_id 生成两条初始化命令。
        注意：这里的 motor_id 字节发送的是“node_id 的低字节位”，但你的适配器/协议示例均在 e8 后面直接放节点字节。
        """
        nid = self._compute_node_id()
        print(f"Initializing {self.motor_type} motor (CAN {self.motor_id}) -> node_id 0x{nid:02X}")

        # init 1
        init_cmd1 = bytes.fromhex(f"41 54 00 07 e8 {nid:02x} 02 01 00 0d 0a")
        self.ser.write(init_cmd1)
        print(f"Sent init1: {init_cmd1.hex(' ').upper()}")
        time.sleep(0.1)

        # init 2
        init_cmd2 = bytes.fromhex(f"41 54 20 07 e8 {nid:02x} 08 00 C4 00 00 00 00 00 00 0d 0a")
        self.ser.write(init_cmd2)
        print(f"Sent init2: {init_cmd2.hex(' ').upper()}")
        time.sleep(0.1)

    def set_position_mode(self):
        """
        位置模式：与原始一致（run_mode = 1）
        data: 05 70 00 00 01 00 00 00
        """
        data = bytes.fromhex("05 70 00 00 01 00 00 00")
        frame = self.build_command(0x90, data)
        self.send_command(frame)
        time.sleep(0.2)

    def set_velocity_mode(self):
        """
        速度模式：run_mode = 2
        data: 05 70 00 00 02 00 00 00
        """
        data = bytes.fromhex("05 70 00 00 02 00 00 00")
        frame = self.build_command(0x90, data)
        self.send_command(frame)
        time.sleep(0.2)

    def enable_motor(self):
        """
        使能：与原始一致
        data: 8字节全0
        """
        data = bytes.fromhex("00 00 00 00 00 00 00 00")
        frame = self.build_command(0x18, data)
        self.send_command(frame)
        time.sleep(0.5)

    def disable_motor(self):
        """
        禁用：与你提供的一致（保持原帧）
        """
        data = bytes.fromhex("01 00 00 00 00 00 00 00")
        frame = self.build_command(0x20, data)
        self.send_command(frame)
        time.sleep(0.5)

    # === 速度模式：可调速度/加速度 ===
    def set_velocity(self, velocity: float):
        """
        写入速度参数（寄存器 0x7024）：
        按“第二套代码”的规则：前缀 24 70 00 00，再补 0x00 + float尾3字节。
        """
        tail3 = self._float_tail3(velocity)
        data = bytes.fromhex("24 70 00 00") + b"\x00" + tail3
        frame = self.build_command(0x90, data)
        self.send_command(frame)
        time.sleep(0.1)

    def set_acceleration(self, acceleration: float):
        """
        写入加速度参数（寄存器 0x7025）：
        按“第二套代码”的规则：前缀 25 70 00 00，再补 0x00 + float尾3字节。
        """
        tail3 = self._float_tail3(acceleration)
        data = bytes.fromhex("25 70 00 00") + b"\x00" + tail3
        frame = self.build_command(0x90, data)
        self.send_command(frame)
        time.sleep(0.1)

    def move_at_velocity(self, velocity: float, acceleration: float):
        """
        速度模式运行（设置速度 + 加速度）
        """
        print(f"[CAN {self.motor_id}] velocity={velocity}, accel={acceleration}")
        self.set_velocity(velocity)
        self.set_acceleration(acceleration)

    # === 位置模式：保留原逻辑（可用作参考/兼容） ===
    def float_to_hex(self, f):
        return struct.pack('<f', f)

    def move_to_position(self, position_rad: float):
        """
        位置模式下设置速度/加速度/位置（沿用你原来的 4字节 float）
        """
        print(f"Moving motor {self.motor_id} to {position_rad} rad (position mode)")
        vel_data = bytes.fromhex("24 70 00 00") + self.float_to_hex(1.0)
        acc_data = bytes.fromhex("25 70 00 00") + self.float_to_hex(5.0)
        pos_data = bytes.fromhex("16 70 00 00") + self.float_to_hex(position_rad)

        self.send_command(self.build_command(0x90, vel_data))
        time.sleep(0.05)
        self.send_command(self.build_command(0x90, acc_data))
        time.sleep(0.05)
        self.send_command(self.build_command(0x90, pos_data))
        time.sleep(0.1)


class RobStrideController:
    """多电机控制器（群控速度模式）"""

    def __init__(self, port: str, baudrate: int = 921600):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        self.motors: dict[int, RobStrideMotor] = {}
        self.running = False
        self.read_thread = None

    def add_motor(
        self,
        motor_id: int,
        motor_type: str = "right",
        mit_mode: bool = False,
        offset_func: Optional[Callable[[float], float]] = None,
        use_side_mapping: bool = True
    ) -> RobStrideMotor:
        m = RobStrideMotor(
            self.ser, motor_id, motor_type, mit_mode, offset_func, use_side_mapping=use_side_mapping
        )
        self.motors[motor_id] = m
        return m

    # 群控：一键切速度模式 + 使能
    def prepare_velocity_mode(self, motor_ids: Optional[Iterable[int]] = None):
        ids = list(motor_ids) if motor_ids is not None else list(self.motors.keys())
        for mid in ids:
            self.motors[mid].set_velocity_mode()
        for mid in ids:
            self.motors[mid].enable_motor()

    # 群控：设置所有电机的速度+加速度
    def move_all_velocity(self, velocity: float, acceleration: float, motor_ids: Optional[Iterable[int]] = None):
        ids = list(motor_ids) if motor_ids is not None else list(self.motors.keys())
        for mid in ids:
            self.motors[mid].move_at_velocity(velocity, acceleration)
            time.sleep(0.02)  # 轻微间隔避免总线碰撞

    # 群控：全部失能
    def disable_all(self, motor_ids: Optional[Iterable[int]] = None):
        ids = list(motor_ids) if motor_ids is not None else list(self.motors.keys())
        for mid in ids:
            self.motors[mid].disable_motor()

    def close(self):
        self.ser.close()


# ========= 示例用法（速度模式，群控 1~4） =========
def main():
    print("Starting multi-motor velocity control...")

    ser = serial.Serial(
        port='COM21',
        baudrate=921600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.5
    )

    # 初始化适配器
    ser.write(bytes.fromhex("41 54 2B 41 54 0D 0A"))
    print("Adapter init sent")
    time.sleep(0.2)

    # 使用控制器（注意：这里直接复用同一个串口句柄）
    controller = RobStrideController(port='COM21')
    controller.ser = ser  # 复用上面的 ser，避免重复打开

    # 根据你的现场规律：left 用原ID，right 用 (ID<<2)+0x08
    # 下面示例：1~4号电机，两左两右（你可以按实际布线调整 motor_type）
    m1 = controller.add_motor(1, motor_type='left',  use_side_mapping=True)
    m2 = controller.add_motor(2, motor_type='right', use_side_mapping=True)
    m3 = controller.add_motor(3, motor_type='left',  use_side_mapping=True)
    m4 = controller.add_motor(4, motor_type='right', use_side_mapping=True)

    try:
        # 切到速度模式并使能
        controller.prepare_velocity_mode()

        # 第一段：速度 2.0，加速度 5.0
        controller.move_all_velocity(velocity=2.0, acceleration=5.0)
        time.sleep(3)

        # 第二段：速度 4.0，加速度 10.0
        controller.move_all_velocity(velocity=4.0, acceleration=10.0)
        time.sleep(3)

        # 第三段：可以单独调不同电机（示例）
        m1.move_at_velocity(velocity=1.5, acceleration=6.0)
        m2.move_at_velocity(velocity=-2.0, acceleration=6.0)
        m3.move_at_velocity(velocity=3.0, acceleration=8.0)
        m4.move_at_velocity(velocity=0.0, acceleration=5.0)
        time.sleep(3)

    except Exception as e:
        print("Error:", e)
        import traceback
        traceback.print_exc()
    finally:
        # 全部禁用，关闭串口
        controller.disable_all()
        controller.close()
        ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()
