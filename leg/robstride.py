import dataclasses
import enum
import math
import struct
import sys
import time
import logging
from typing import List, Union, Optional, Dict, Type, Tuple
import can

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("RS04Motor")

# 检查操作系统
IS_WINDOWS = sys.platform.startswith('win')

class RunMode(enum.Enum):
    Operation = 0
    Position = 1
    Speed = 2
    Current = 3
    PositionCSP = 5  # 添加 CSP 模式

class MotorMsg(enum.Enum):
    Info = 0
    Control = 1
    Feedback = 2
    Enable = 3
    Disable = 4
    ZeroPos = 6
    SetID = 7
    ReadParam = 17
    WriteParam = 18

class MotorMode(enum.Enum):
    Reset = 0
    Calibration = 1
    Run = 2

class MotorError(enum.Enum):
    Undervoltage = 1
    Overcurrent = 2
    Overtemp = 4
    MagneticEncodingFault = 8
    HallEncodingFault = 16
    Uncalibrated = 32

@dataclasses.dataclass
class FeedbackResp:
    servo_id: int
    errors: List[MotorError]
    mode: MotorMode
    angle: float
    velocity: float
    torque: float
    temp: float

# 参数映射表
params = [
    ('run_mode', 0x7005, RunMode),
    ('iq_ref', 0x7006, float),
    ('spd_ref', 0x700A, float),
    ('limit_torque', 0x700B, float),
    ('cur_kp', 0x7010, float),
    ('cur_ki', 0x7011, float),
    ('cur_fit_gain', 0x7014, float),
    ('loc_ref', 0x7016, float),
    ('limit_spd', 0x7017, float),
    ('limit_cur', 0x7018, float),
    ('mechpos', 0x7019, float),
    ('iqf', 0x701A, float),
    ('mechvel', 0x701B, float),
    ('vbus', 0x701C, float),
    ('loc_kp', 0x701E, float),
    ('spd_kp', 0x701F, float),
    ('spd_ki', 0x7020, float),
    ('spd_filt_gain', 0x7021, float),
]

# 创建参数映射字典
param_ids_by_name: Dict[str, int] = {}
param_types_by_id: Dict[int, Type] = {}
for name, id_, type_ in params:
    param_ids_by_name[name] = id_
    param_types_by_id[id_] = type_

class Client:
    def __init__(self, bus: can.BusABC, retry_count=3, recv_timeout=0.2, host_can_id=0xAA):
        self.bus = bus
        self.retry_count = retry_count
        self.recv_timeout = recv_timeout
        self.host_can_id = host_can_id
        self._recv_count = 0
        self._recv_error_count = 0
        self._last_message = None
        self._active_motors = set()
    
    def enable(self, motor_id: int, motor_model=1) -> FeedbackResp:
        """使能电机"""
        self.send_message(MotorMsg.Enable, motor_id)
        resp = self._recv(expected_msg_type=MotorMsg.Feedback.value)
        return self._parse_feedback_resp(resp, motor_id, motor_model)
    
    def disable(self, motor_id: int, motor_model=1) -> FeedbackResp:
        """禁用电机"""
        self.send_message(MotorMsg.Disable, motor_id)
        resp = self._recv(expected_msg_type=MotorMsg.Feedback.value)
        return self._parse_feedback_resp(resp, motor_id, motor_model)
    
    def set_zero_position(self, motor_id: int, motor_model=1) -> FeedbackResp:
        """设置零点位置"""
        self.send_message(MotorMsg.ZeroPos, motor_id)
        resp = self._recv(expected_msg_type=MotorMsg.Feedback.value)
        return self._parse_feedback_resp(resp, motor_id, motor_model)
    
    def update_id(self, motor_id: int, new_motor_id: int):
        """更新电机ID"""
        id_data_1 = self.host_can_id | (new_motor_id << 8)
        self.send_message(MotorMsg.SetID, motor_id, id_data_1=id_data_1)
        self._recv(expected_msg_type=MotorMsg.Info.value)  # 期待信息帧
    
    def read_param(self, motor_id: int, param_id: Union[int, str]) -> Union[float, RunMode]:
        """读取参数值"""
        param_id = self._normalize_param_id(param_id)
        
        # 构建读取请求数据
        data = struct.pack('<Hxx', param_id) + bytes(4)  # 索引 + 保留 + 空值
        self.send_message(MotorMsg.ReadParam, motor_id, data=data)
        resp = self._recv(expected_msg_type=MotorMsg.ReadParam.value)
        
        # 验证响应
        self._parse_and_validate_resp_arbitration_id(resp, MotorMsg.ReadParam.value, motor_id)
        
        # 解析参数ID
        resp_param_id = struct.unpack('<H', resp.data[:2])[0]
        if resp_param_id != param_id:
            raise Exception(f'Invalid param id received: {resp_param_id:#x}, expected: {param_id:#x}')
        
        # 根据参数类型解析值
        param_type = param_types_by_id.get(param_id, float)
        
        if param_type == RunMode:
            value = RunMode(resp.data[4])
        else:
            # 浮点数值在数据区的最后4个字节
            value = struct.unpack('<f', resp.data[4:8])[0]
        
        return value
    
    def write_param(self, motor_id: int, param_id: Union[int, str], 
                    param_value: Union[float, RunMode, int], 
                    motor_model=1) -> FeedbackResp:
        """写入参数值"""
        param_id = self._normalize_param_id(param_id)
        
        # 构建写入请求数据
        data = bytearray(8)
        struct.pack_into('<H', data, 0, param_id)  # 参数ID
        
        if isinstance(param_value, RunMode):
            data[4] = param_value.value
        elif isinstance(param_value, int):
            data[4] = param_value
        else:
            struct.pack_into('<f', data, 4, param_value)
        
        self.send_message(MotorMsg.WriteParam, motor_id, data=data)
        resp = self._recv(expected_msg_type=MotorMsg.Feedback.value)
        
        return self._parse_feedback_resp(resp, motor_id, motor_model)
    
    def get_feedback(self, motor_id: int, motor_model=1) -> FeedbackResp:
        """获取电机反馈数据"""
        # 发送一个空的控制消息来请求反馈
        self.send_message(MotorMsg.Control, motor_id)
        resp = self._recv(expected_msg_type=MotorMsg.Feedback.value)
        return self._parse_feedback_resp(resp, motor_id, motor_model)
    
    def send_message(self, msg_type: MotorMsg, motor_id: int, 
                     id_data_1: Optional[int] = None, data: bytes = bytes(8)):
        """发送消息到电机"""
        if id_data_1 is None:
            id_data_1 = self.host_can_id
        
        arb_id = motor_id + (id_data_1 << 8) + (msg_type.value << 24)
        message = can.Message(
            arbitration_id=arb_id,
            data=data,
            is_extended_id=True
        )
        self.bus.send(message)
        self._last_message = message
        logger.debug(f"Sent message: {message}")
    
    def error_rate(self) -> float:
        """计算错误率"""
        if self._recv_count == 0:
            return 0.0
        return self._recv_error_count / self._recv_count
    
    def _recv(self, expected_msg_type: Optional[int] = None) -> can.Message:
        """接收消息，可选择指定期望的消息类型"""
        retry_count = 0
        start_time = time.time()
        
        while retry_count <= self.retry_count:
            # 计算剩余超时时间
            elapsed = time.time() - start_time
            if elapsed >= self.recv_timeout:
                break
            remaining_timeout = self.recv_timeout - elapsed
            
            # 接收消息
            resp = self.bus.recv(remaining_timeout)
            self._recv_count += 1
            
            if resp is None:
                retry_count += 1
                continue
            
            if resp.is_error_frame:
                self._recv_error_count += 1
                logger.warning(f"Received error frame: {resp}")
                continue
            
            # 如果指定了期望的消息类型，检查是否匹配
            if expected_msg_type is not None:
                msg_type, _, _ = self._parse_resp_abitration_id(resp.arbitration_id)
                if msg_type != expected_msg_type:
                    logger.debug(f"Received unexpected msg_type: {msg_type}, expected: {expected_msg_type}")
                    continue
            
            logger.debug(f"Received message: {resp}")
            return resp
        
        raise Exception(f'No valid response received after {self.retry_count} retries')
    
    def _parse_resp_abitration_id(self, aid: int) -> Tuple[int, int, int]:
        """解析仲裁ID"""
        msg_type = (aid >> 24) & 0x1F
        msg_motor_id = (aid >> 8) & 0xFF
        host_id = aid & 0xFF
        return msg_type, msg_motor_id, host_id
    
    def _parse_and_validate_resp_arbitration_id(self, resp: can.Message, 
                                               expected_msg_type: int, 
                                               expected_motor_id: int):
        """验证响应ID"""
        msg_type, msg_motor_id, host_id = self._parse_resp_abitration_id(resp.arbitration_id)
        
        if msg_type != expected_msg_type:
            raise Exception(f'Invalid msg_type: expected {expected_msg_type}, got {msg_type}')
        
        if host_id != self.host_can_id:
            raise Exception(f'Invalid host CAN ID: expected {self.host_can_id:#x}, got {host_id:#x}')
        
        if msg_motor_id != expected_motor_id:
            raise Exception(f'Invalid motor ID: expected {expected_motor_id}, got {msg_motor_id}')
        
        return msg_type, msg_motor_id, host_id
    
    def _parse_feedback_resp(self, resp: can.Message, motor_id: int, motor_model: int) -> FeedbackResp:
        """解析反馈响应"""
        try:
            # 首先验证消息类型
            msg_type, msg_motor_id, host_id = self._parse_resp_abitration_id(resp.arbitration_id)
            
            if msg_type != MotorMsg.Feedback.value:
                logger.warning(f"Expected feedback message (type 2), got type {msg_type}")
            
            # 提取错误位 (6位)
            error_bits = (resp.arbitration_id >> 16) & 0x3F
            errors = []
            
            # 检查每个错误位
            for error in MotorError:
                if error.value & error_bits:
                    errors.append(error)
            
            # 提取模式位 (2位)
            mode_value = (resp.arbitration_id >> 22) & 0x03
            try:
                mode = MotorMode(mode_value)
            except ValueError:
                mode = MotorMode.Reset
                logger.warning(f"Unknown mode value: {mode_value}")
            
            # 解析数据字段
            if len(resp.data) >= 8:
                angle_raw = struct.unpack('>H', resp.data[0:2])[0]
                angle = (angle_raw / 65535.0 * 8 * math.pi) - 4 * math.pi
                
                velocity_raw = struct.unpack('>H', resp.data[2:4])[0]
                velocity_range = 88.0 if motor_model == 1 else 30.0
                velocity = (velocity_raw / 65535.0 * 2 * velocity_range) - velocity_range
                
                torque_raw = struct.unpack('>H', resp.data[4:6])[0]
                torque_range = 34.0 if motor_model == 1 else 240.0
                torque = (torque_raw / 65535.0 * 2 * torque_range) - torque_range
                
                temp_raw = struct.unpack('>H', resp.data[6:8])[0]
                temp = temp_raw / 10.0
            else:
                # 数据长度不足，使用默认值
                logger.warning(f"Insufficient data length: {len(resp.data)} bytes")
                angle, velocity, torque, temp = 0.0, 0.0, 0.0, 0.0
            
            return FeedbackResp(
                servo_id=msg_motor_id,
                errors=errors,
                mode=mode,
                angle=angle,
                velocity=velocity,
                torque=torque,
                temp=temp
            )
        except Exception as e:
            logger.error(f"Error parsing feedback response: {e}")
            logger.error(f"Response data: {resp}")
            raise
    
    def _normalize_param_id(self, param_id: Union[int, str]) -> int:
        """标准化参数ID"""
        if isinstance(param_id, str):
            if param_id not in param_ids_by_name:
                raise ValueError(f"Unknown parameter name: {param_id}")
            return param_ids_by_name[param_id]
        return param_id
    
    def shutdown(self):
        """安全关闭所有资源"""
        # 禁用所有活跃的电机
        for motor_id in list(self._active_motors):
            try:
                self.disable(motor_id)
            except Exception as e:
                logger.error(f"Error disabling motor {motor_id}: {e}")
        
        # 关闭总线
        if hasattr(self.bus, 'shutdown'):
            try:
                self.bus.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down bus: {e}")


# ================== Windows 专用支持 ==================
if IS_WINDOWS:
    import serial
    import binascii
    from collections import deque
    
    class WindowsCANAdapter:
        """Windows 平台的 CAN 适配器实现（通过串行端口）"""
        def __init__(self, port='COM21', baudrate=921600):
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.1
            )
            self.rx_buffer = deque()
            self._initialize_adapter()
        
        def _initialize_adapter(self):
            """初始化适配器"""
            # 发送初始化序列
            init_sequence = [
                b'ATZ\r',       # 复位适配器
                b'ATE0\r',      # 关闭回显
                b'ATL0\r',      # 关闭换行符
                b'ATS0=1000000\r',  # 设置波特率1Mbps
                b'ATMA\r'       # 进入监控模式
            ]
            
            for cmd in init_sequence:
                self.ser.write(cmd)
                time.sleep(0.1)
                response = self.ser.read_all()
                if response:
                    logger.debug(f"Init response: {binascii.hexlify(response).decode()}")
        
        def send(self, msg: can.Message):
            """发送 CAN 消息"""
            # 构建标准SLCAN帧: tiiildd...
            # t: 帧类型 (t = 标准帧, T = 扩展帧)
            # iii: 3位十六进制ID (标准帧) 或 8位十六进制ID (扩展帧)
            # l: 数据长度 (0-8)
            # dd: 数据字节 (十六进制)
            
            # 使用扩展帧格式
            frame_id = f"{msg.arbitration_id:08X}"
            data_hex = ''.join([f"{b:02X}" for b in msg.data])
            frame = f"T{frame_id}{len(msg.data):X}{data_hex}\r"
            
            self.ser.write(frame.encode())
            logger.debug(f"Sent frame: {frame.strip()}")
        
        def recv(self, timeout=0.5) -> Optional[can.Message]:
            """接收 CAN 消息"""
            start_time = time.time()
            while time.time() - start_time < timeout:
                # 读取所有可用数据
                while self.ser.in_waiting:
                    byte = self.ser.read(1)
                    if byte:
                        self.rx_buffer.append(byte[0])
                
                # 尝试解析帧
                if self.rx_buffer and self.rx_buffer[0] in [b't', b'T', b'r', b'R']:
                    # 获取完整行
                    line = bytearray()
                    while self.rx_buffer and self.rx_buffer[0] != 0x0D:  # 直到回车符
                        line.append(self.rx_buffer.popleft())
                    
                    # 移除回车符
                    if self.rx_buffer and self.rx_buffer[0] == 0x0D:
                        self.rx_buffer.popleft()
                    
                    if not line:
                        continue
                    
                    # 解析帧
                    try:
                        frame_type = chr(line[0])
                        # 扩展帧
                        if frame_type == 'T':
                            arb_id = int(line[1:9].decode(), 16)
                            data_len = int(chr(line[9]), 16)
                            data = bytes.fromhex(line[10:10+data_len*2].decode())
                            return can.Message(
                                arbitration_id=arb_id,
                                data=data,
                                is_extended_id=True
                            )
                    except Exception as e:
                        logger.error(f"Error parsing frame: {line}, error: {e}")
            
            return None
        
        def shutdown(self):
            """关闭连接"""
            try:
                self.ser.close()
            except Exception as e:
                logger.error(f"Error closing serial port: {e}")

    # Windows 上创建总线的辅助函数
    def create_windows_can_bus(port='COM21', baudrate=921600):
        """为 Windows 创建 CAN 总线适配器"""
        adapter = WindowsCANAdapter(port=port, baudrate=baudrate)
        # 使用虚拟总线接口
        return can.Bus(bustype='virtual', channel=adapter, receive_own_messages=False)


# ================== 使用示例 ==================
def create_can_bus():
    """创建适合当前操作系统的 CAN 总线"""
    if IS_WINDOWS:
        # Windows 使用我们的自定义适配器
        return create_windows_can_bus(port='COM21', baudrate=921600)
    else:
        # Linux 和其他系统使用 socketcan
        return can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)

def example_usage():
    """使用示例"""
    bus = None
    rs_client = None
    
    try:
        # 创建适合当前操作系统的总线
        logger.info("Creating CAN bus...")
        bus = create_can_bus()
        
        # 创建客户端
        logger.info("Creating motor client...")
        rs_client = Client(bus, host_can_id=0xAA)
        
        logger.info("Starting motor control example...")
        motor_id = 1
        
        # 设置运行模式为位置模式
        logger.info("Setting motor to position mode...")
        rs_client.write_param(motor_id, 'run_mode', RunMode.Position)
        
        # 使能电机
        logger.info("Enabling motor...")
        feedback = rs_client.enable(motor_id)
        logger.info(f"Motor enabled. Current position: {feedback.angle:.2f} rad")
        
        # 移动到零位
        logger.info("Moving to zero position...")
        rs_client.write_param(motor_id, 'loc_ref', 0.0)
        
        # 等待移动完成
        time.sleep(2)
        
        # 获取当前位置
        position = rs_client.read_param(motor_id, 'mechpos')
        logger.info(f"Current position: {position:.2f} rad")
        
        # 移动到新位置
        target_pos = 1.57  # π/2 (90度)
        logger.info(f"Moving to {target_pos:.2f} rad...")
        rs_client.write_param(motor_id, 'loc_ref', target_pos)
        
        # 等待移动完成
        time.sleep(2)
        
        # 获取反馈信息
        feedback = rs_client.get_feedback(motor_id)
        logger.info(f"Position: {feedback.angle:.2f} rad, Temp: {feedback.temp:.1f}°C")
        
        # 设置零点
        logger.info("Setting zero position...")
        rs_client.set_zero_position(motor_id)
        
    except Exception as e:
        logger.error(f"Error: {str(e)}", exc_info=True)
    finally:
        # 禁用电机并关闭总线
        if rs_client:
            logger.info("Disabling motor...")
            try:
                rs_client.disable(1)
            except Exception as e:
                logger.error(f"Error disabling motor: {e}")
            
            logger.info("Shutting down client...")
            rs_client.shutdown()
        
        if bus:
            logger.info("Shutting down bus...")
            try:
                bus.shutdown()
            except Exception as e:
                logger.error(f"Error shutting down bus: {e}")
        
        logger.info("Example completed")

if __name__ == "__main__":
    example_usage()