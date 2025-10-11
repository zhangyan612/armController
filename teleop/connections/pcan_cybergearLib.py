import struct
import can
import logging
import enum
import math


class CANMotorController:
    PARAM_TABLE = {
        "motorOverTemp": {"feature_code": 0x200D, "type": "int16"},
        "overTempTime": {"feature_code": 0x200E, "type": "int32"},
        "limit_torque": {"feature_code": 0x2007, "type": "float"},
        "cur_kp": {"feature_code": 0x2012, "type": "float"},
        "cur_ki": {"feature_code": 0x2013, "type": "float"},
        "spd_kp": {"feature_code": 0x2014, "type": "float"},
        "spd_ki": {"feature_code": 0x2015, "type": "float"},
        "loc_kp": {"feature_code": 0x2016, "type": "float"},
        "spd_filt_gain": {"feature_code": 0x2017, "type": "float"},
        "limit_spd": {"feature_code": 0x2018, "type": "float"},
        "limit_cur": {"feature_code": 0x2019, "type": "float"},
    }

    PARAMETERS = {
        "run_mode": {"index": 0x7005, "format": "u8"},
        "iq_ref": {"index": 0x7006, "format": "f"},
        "spd_ref": {"index": 0x700A, "format": "f"},
        "limit_torque": {"index": 0x700B, "format": "f"},
        "cur_kp": {"index": 0x7010, "format": "f"},
        "cur_ki": {"index": 0x7011, "format": "f"},
        "cur_filt_gain": {"index": 0x7014, "format": "f"},
        "loc_ref": {"index": 0x7016, "format": "f"},
        "limit_spd": {"index": 0x7017, "format": "f"},
        "limit_cur": {"index": 0x7018, "format": "f"},
        "mechPos": {"index": 0x7019, "format": "f"},      # Load end mechanical angle (rad)
        "iqf": {"index": 0x701A, "format": "f"},          # iq filter value (A)
        "mechVel": {"index": 0x701B, "format": "f"},      # Load end speed (rad/s)
        "VBUS": {"index": 0x701C, "format": "f"},         # bus voltage (V)
        "rotation": {"index": 0x701D, "format": "s16"},   # Number of turns
        "loc_kp": {"index": 0x701E, "format": "f"},       # kp of position
        "spd_kp": {"index": 0x701F, "format": "f"},       # Speed kp
        "spd_ki": {"index": 0x7020, "format": "f"},       # Speed ki
    }
    
    TWO_BYTES_BITS = 16

    def __init__(self, bus, motor_id=127, main_can_id=254):
        """
        初始化CAN电机控制器。

        参数:
        bus: CAN总线对象。
        motor_id: 电机的CAN ID。
        main_can_id: 主CAN ID。
        """
        self.bus = bus
        self.MOTOR_ID = motor_id
        self.MAIN_CAN_ID = main_can_id
        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -30.0
        self.V_MAX = 30.0
        self.T_MIN = -12.0
        self.T_MAX = 12.0
        self.KP_MIN, self.KP_MAX = 0.0, 500.0  # 0.0 ~ 500.0
        self.KD_MIN, self.KD_MAX = 0.0, 5.0  # 0.0 ~ 5.0

    # 通信类型
    class CmdModes:
        GET_DEVICE_ID = 0
        MOTOR_CONTROL = 1
        MOTOR_FEEDBACK = 2
        MOTOR_ENABLE = 3
        MOTOR_STOP = 4
        SET_MECHANICAL_ZERO = 6
        SET_MOTOR_CAN_ID = 7
        PARAM_TABLE_WRITE = 8
        SINGLE_PARAM_READ = 17
        SINGLE_PARAM_WRITE = 18
        FAULT_FEEDBACK = 21
    
    # 控制模式
    class RunModes(enum.Enum):
        CONTROL_MODE = 0 # 运控模式
        POSITION_MODE = 1 # 位置模式
        SPEED_MODE = 2 # 速度模式
        CURRENT_MODE = 3 # 电流模式

    def _float_to_uint(self, x, x_min, x_max, bits):
        """
        将浮点数转换为无符号整数。

        参数:
        x: 输入的浮点数。
        x_min: 可接受的最小浮点数。
        x_max: 可接受的最大浮点数。
        bits: 输出无符号整数的位数。

        返回:
        转换后的无符号整数。
        """
        span = x_max - x_min
        offset = x_min
        x = max(min(x, x_max), x_min)  # Clamp x to the range [x_min, x_max]
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def _uint_to_float(self, x, x_min, x_max, bits):
        """
        将无符号整数转换为浮点数。

        参数:
        x: 输入的无符号整数。
        x_min: 可接受的最小浮点数。
        x_max: 可接受的最大浮点数。
        bits: 输入无符号整数的位数。

        返回:
        转换后的浮点数。
        """
        span = (1 << bits) - 1
        offset = x_max - x_min
        x = max(min(x, span), 0)  # Clamp x to the range [0, span]
        return offset * x / span + x_min

    def _linear_mapping(
        self, value, value_min, value_max, target_min=0, target_max=65535
    ):
        """
        对输入值进行线性映射。

        参数:
        value: 输入值。
        value_min: 输入值的最小界限。
        value_max: 输入值的最大界限。
        target_min: 输出值的最小界限。
        target_max: 输出值的最大界限。

        返回:
        映射后的值。
        """
        return int(
            (value - value_min) / (value_max - value_min) * (target_max - target_min)
            + target_min
        )

    def format_data(self, data=[], format="f f", type="decode"):
        """
        对数据进行编码或解码。

        参数:
        data: 输入的数据列表。
        format: 数据的格式。
        type: "encode" 或 "decode", 表示是进行编码还是解码。

        返回:
        编码或解码后的数据。
        """
        format_list = format.split()
        rdata = []
        if type == "decode":
            p = 0
            for f in format_list:
                s_f = []
                if f == "f":
                    s_f = [4, "f"]
                elif f == "u16":
                    s_f = [2, "H"]
                elif f == "s16":
                    s_f = [2, "h"]
                elif f == "u32":
                    s_f = [4, "I"]
                elif f == "s32":
                    s_f = [4, "i"]
                elif f == "u8":
                    s_f = [1, "B"]
                elif f == "s8":
                    s_f = [1, "b"]
                ba = bytearray()
                if len(s_f) == 2:
                    for i in range(s_f[0]):
                        ba.append(data[p])
                        p = p + 1
                    rdata.append(struct.unpack(s_f[1], ba)[0])
                else:
                    logging.info("unknown format in format_data(): " + f)
                    return []
            return rdata
        elif type == "encode" and len(format_list) == len(data):
            for i in range(len(format_list)):
                f = format_list[i]
                s_f = []
                if f == "f":
                    s_f = [4, "f"]
                elif f == "u16":
                    s_f = [2, "H"]
                elif f == "s16":
                    s_f = [2, "h"]
                elif f == "u32":
                    s_f = [4, "I"]
                elif f == "s32":
                    s_f = [4, "i"]
                elif f == "u8":
                    s_f = [1, "B"]
                elif f == "s8":
                    s_f = [1, "b"]
                if f != "f":
                    data[i] = int(data[i])
                if len(s_f) == 2:
                    bs = struct.pack(s_f[1], data[i])
                    for j in range(s_f[0]):
                        rdata.append(bs[j])
                else:
                    logging.info("unkown format in format_data(): " + f)
                    return []
            if len(rdata) < 4:
                for i in range(4 - len(rdata)):
                    rdata.append(0x00)
            return rdata

    def pack_to_8bytes(self, target_angle, target_velocity, Kp, Kd):
        """
        定义打包data1函数, 将控制参数打包为8字节的数据。

        参数:
        target_angle: 目标角度。
        target_velocity: 目标速度。
        Kp: 比例增益。
        Kd: 微分增益。

        返回:
        8字节的数据。
        """
        # 对输入变量进行线性映射
        target_angle_mapped = self._linear_mapping(
            target_angle, self.P_MIN, self.P_MAX)
        target_velocity_mapped = self._linear_mapping(
            target_velocity, self.V_MIN, self.V_MAX
        )
        Kp_mapped = self._linear_mapping(Kp, self.KP_MIN, self.KP_MAX)
        Kd_mapped = self._linear_mapping(Kd, self.KD_MIN, self.KD_MAX)

        # 使用Python的struct库进行打包
        # 使用H表示无符号短整数(2字节), 共需要8字节
        data1_bytes = struct.pack(
            "HHHH", target_angle_mapped, target_velocity_mapped, Kp_mapped, Kd_mapped
        )
        data1 = [b for b in data1_bytes]
        return data1

    def send_receive_can_message(self, cmd_mode, data2, data1, timeout=200):
        """
        发送CAN消息并接收响应。

        参数:
        cmd_mode: 命令模式。
        data2: 数据区2。
        data1: 要发送的数据字节。
        timeout: 发送消息的超时时间(默认为200ms)。

        返回:
        一个元组, 包含接收到的消息数据和接收到的消息仲裁ID(如果有)。
        """
        # Calculate the arbitration ID
        arbitration_id = (cmd_mode << 24) | (data2 << 8) | self.MOTOR_ID
        message = can.Message(
            arbitration_id=arbitration_id, data=data1, is_extended_id=True
        )

        # Send the CAN message
        try:
            self.bus.send(message)
        except:
            logging.info("Failed to send the message.")
            return None, None

        # Output details of the sent message
        logging.debug(
            f"Sent message with ID {hex(arbitration_id)}, data: {data1}")

        # Receive a CAN message with a 1-second timeout
        # 1-second timeout for receiving
        received_msg = self.bus.recv(timeout=1)
        if received_msg:
            return received_msg.data, received_msg.arbitration_id
        else:
            return None, None

    def parse_received_msg(self, data, arbitration_id):
        """
        解析接收到的CAN消息。

        参数:
        data: 接收到的数据。
        arbitration_id: 接收到的消息的仲裁ID。

        返回:
        一个元组, 包含电机的CAN ID、位置(rad)、速度(rad/s)、力矩(Nm)、温度(摄氏度)。
        """
        if data is not None:
            logging.debug(f"Received message with ID {hex(arbitration_id)}")
            # 解析电机CAN ID
            motor_can_id = (arbitration_id >> 8) & 0xFF

            pos = self._uint_to_float(
                (data[0] << 8) + data[1], self.P_MIN, self.P_MAX, self.TWO_BYTES_BITS
            )
            vel = self._uint_to_float(
                (data[2] << 8) + data[3], self.V_MIN, self.V_MAX, self.TWO_BYTES_BITS
            )
            torque = self._uint_to_float(
                (data[4] << 8) + data[5], self.T_MIN, self.T_MAX, self.TWO_BYTES_BITS
            )
            # 解析温度数据
            temperature_raw = (data[6] << 8) + data[7]
            temperature_celsius = temperature_raw / 10.0
            
            logging.info(
                f"Motor CAN ID: {motor_can_id}, pos: {pos:.2f} rad, vel: {vel:.2f} rad/s, "
                f"torque: {torque:.2f} Nm, temperature: {temperature_celsius:.1f} °C"
            )
            
            return motor_can_id, pos, vel, torque, temperature_celsius
        else:
            logging.info("No message received within the timeout period.")
            return None, None, None, None, None

    def clear_can_rx(self, timeout=10):
        """
        清除接收缓冲区中的所有现有消息。

        参数:
        timeout: 等待清除操作的时间（单位：毫秒）。
        """
        timeout_seconds = timeout / 1000.0  # Convert to seconds
        while True:
            received_msg = self.bus.recv(timeout=timeout_seconds)
            if received_msg is None:
                break
            logging.info(
                f"Cleared message with ID {hex(received_msg.arbitration_id)}")

    def _read_single_param(self, index, format="u32"):
        """
        读取单个参数。

        参数:
        index: 参数索引。
        format: 数据格式。

        返回:
        参数值。
        """
        # For reading, we send the index and zeros for the value part
        data1 = [b for b in struct.pack("<I", index)] + [0, 0, 0, 0]

        self.clear_can_rx()  # 空CAN接收缓存, 避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.SINGLE_PARAM_READ,
            data2=self.MAIN_CAN_ID,
            data1=data1,
        )
        
        if received_msg_data:
            # Parse the parameter value from response
            # The response should contain the parameter value in the data
            value_data = received_msg_data[4:8]  # Last 4 bytes contain the value
            
            if format == "f":
                # Float type
                value = struct.unpack('<f', bytes(value_data))[0]
            elif format == "u8":
                # Unsigned 8-bit
                value = value_data[0]
            elif format == "s16":
                # Signed 16-bit
                value = struct.unpack('<h', bytes(value_data[:2]))[0]
            elif format == "u16":
                # Unsigned 16-bit
                value = struct.unpack('<H', bytes(value_data[:2]))[0]
            elif format == "s32":
                # Signed 32-bit
                value = struct.unpack('<i', bytes(value_data))[0]
            elif format == "u32":
                # Unsigned 32-bit
                value = struct.unpack('<I', bytes(value_data))[0]
            else:
                logging.info(f"Unknown format: {format}")
                return None
                
            return value
        else:
            return None

    def read_single_param(self, param_name):
        """
        通过参数名称读取单个参数。

        参数:
        param_name: 参数名称。

        返回:
        参数值。
        """
        param_info = self.PARAMETERS.get(param_name)
        if param_info is None:
            logging.info(f"Unknown parameter name: {param_name}")
            return None

        index = param_info["index"]
        format = param_info["format"]

        return self._read_single_param(index=index, format=format)

    def _write_single_param(self, index, value, format="u32"):
        """
        写入单个参数。

        参数:
        index: 参数索引。
        value: 要设置的值。
        format: 数据格式。

        返回:
        解析后的接收消息。
        """
        encoded_data = self.format_data(
            data=[value], format=format, type="encode")
        data1 = [b for b in struct.pack("<I", index)] + encoded_data

        self.clear_can_rx()  # 空CAN接收缓存, 避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.SINGLE_PARAM_WRITE,
            data2=self.MAIN_CAN_ID,
            data1=data1,
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def write_single_param(self, param_name, value):
        """
        通过参数名称写入单个参数。

        参数:
        param_name: 参数名称。
        value: 要设置的值。

        返回:
        写入操作的结果。
        """
        param_info = self.PARAMETERS.get(param_name)
        if param_info is None:
            logging.info(f"Unknown parameter name: {param_name}")
            return

        index = param_info["index"]
        format = param_info["format"]

        return self._write_single_param(index=index, value=value, format=format)

    def write_param_table(self, param_name, value):
        """
        写入参数表。

        参数:
        param_name: 参数名称。
        value: 要设置的值。

        返回:
        接收到的消息数据和仲裁ID。
        """
        # Get the parameter info from PARAM_TABLE
        param_info = self.PARAM_TABLE.get(param_name)
        if param_info is None:
            logging.info(f"Unknown parameter name: {param_name}")
            return None, None, None

        feature_code = param_info["feature_code"]
        param_type = param_info["type"]

        # Type code mapping
        type_code_mapping = {
            "float": 0x06,
            "int16": 0x03,
            "int32": 0x04,
        }

        type_code = type_code_mapping.get(param_type)
        if type_code is None:
            logging.info(f"Unknown parameter type: {param_type}")
            return None, None, None

        # Encode the value based on the type
        format_mapping = {
            "float": "f",
            "int16": "s16",
            "int32": "s32",
        }

        format = format_mapping.get(param_type)
        encoded_value = self.format_data(
            data=[value], format=format, type="encode")

        # Construct data1
        data1 = [b for b in struct.pack("<H", feature_code)]
        data1.extend([type_code, 0x00])
        data1.extend(encoded_value)

        # Clear the CAN receive buffer
        self.clear_can_rx()

        # Send the CAN message
        cmd_mode = self.CmdModes.PARAM_TABLE_WRITE
        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=cmd_mode,
            data2=self.MAIN_CAN_ID,
            data1=data1,
        )

        return received_msg_data, received_msg_arbitration_id

    def set_0_pos(self):
        """
        设置电机的机械零点。

        返回:
        解析后的接收消息。
        """
        self.clear_can_rx()  # 清空CAN接收缓存, 避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.SET_MECHANICAL_ZERO,
            data2=self.MAIN_CAN_ID,
            data1=[1],
        )

        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def enable(self):
        """
        使能运行电机。

        返回:
        解析后的接收消息。
        """
        self.clear_can_rx(0)  # 清空CAN接收缓存, 避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.MOTOR_ENABLE, data2=self.MAIN_CAN_ID, data1=[]
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def disable(self):
        """
        停止运行电机。

        返回:
        解析后的接收消息。
        """
        self.clear_can_rx(0)  # 空CAN接收缓存, 避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.MOTOR_STOP,
            data2=self.MAIN_CAN_ID,
            data1=[0, 0, 0, 0, 0, 0, 0, 0],
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def set_run_mode(self, mode):
        """
        设置运行模式。

        参数:
        mode: 运行模式，应为 RunModes 枚举的一个实例。

        返回:
        写入操作的结果。
        """
        if not isinstance(mode, self.RunModes):
            raise ValueError(
                f"Invalid mode: {mode}. Must be an instance of RunModes enum.")
        return self.write_single_param("run_mode", value=mode.value)

    def set_motor_position_control(self, limit_spd, loc_ref):
        """
        位置模式下设置电机的位置控制参数。

        参数:
        limit_spd: 电机的最大速度。
        loc_ref: 电机的目标位置。

        返回:
        None。
        """
        # 设置电机最大速度
        self.write_single_param(param_name="limit_spd", value=limit_spd)
        # 设置电机目标位置
        self.write_single_param(param_name="loc_ref", value=loc_ref)

    def send_motor_control_command(
        self, torque, target_angle, target_velocity, Kp, Kd
    ):
        """
        运控模式下发送电机控制指令。

        参数:
        torque: 扭矩。
        target_angle: 目标角度。
        target_velocity: 目标速度。
        Kp: 比例增益。
        Kd: 导数增益。

        返回:
        解析后的接收消息。
        """

        # 生成29位的仲裁ID的组成部分
        cmd_mode = self.CmdModes.MOTOR_CONTROL
        torque_mapped = self._linear_mapping(
            torque, -12.0, 12.0, target_min=0, target_max=65535)
        data2 = torque_mapped

        # 将实际值映射到消息值
        target_angle_mapped = self._linear_mapping(
            target_angle, -4 * math.pi, 4 * math.pi)
        target_velocity_mapped = self._linear_mapping(
            target_velocity, -30.0, 30.0)
        Kp_mapped = self._linear_mapping(Kp, 0.0, 500.0)
        Kd_mapped = self._linear_mapping(Kd, 0.0, 5.0)

        # 创建8字节的数据区
        data1_bytes = struct.pack(
            "HHHH", target_angle_mapped, target_velocity_mapped, Kp_mapped, Kd_mapped
        )
        data1 = [b for b in data1_bytes]

        # 使用send_receive_can_message方法发送消息并接收响应
        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=cmd_mode,
            data2=data2,
            data1=data1
        )

        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)


if __name__ == "__main__":
    import time
    import can

    try:
        # Initialize CAN bus
        bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
        print("CAN bus initialized successfully")
        
        # Create motor controller instance
        motor = CANMotorController(bus, motor_id=6, main_can_id=0)
        
        print("=== Testing Parameter Reading ===")
        
        # Test reading various parameters
        parameters_to_test = [
            "run_mode",
            "mechPos",    # Mechanical position
            "mechVel",    # Mechanical velocity  
            "VBUS",       # Bus voltage
            "rotation",   # Number of turns
            "iqf",        # iq filter value
        ]
        
        print("\n1. Reading initial parameters...")
        for param_name in parameters_to_test:
            try:
                value = motor.read_single_param(param_name)
                if value is not None:
                    print(f"  {param_name}: {value}")
                else:
                    print(f"  {param_name}: Failed to read")
            except Exception as e:
                print(f"  {param_name}: Error - {e}")
        
        # print("\n2. Testing motor feedback (position reading)...")
        # # Test motor feedback to see actual position values
        # try:
        #     # Enable motor first
        #     motor.enable()
        #     print("✓ Motor enabled for testing")
            
        #     # Read position multiple times to see if it changes
        #     print("Reading position feedback (move motor manually to see changes):")
        #     for i in range(10):
        #         # Use write_single_param to get motor status feedback
        #         status = motor.write_single_param("run_mode", 0)
        #         if status and status[1] is not None:
        #             print(f"  Reading {i+1}: Position = {status[1]:.3f} rad, "
        #                   f"Velocity = {status[2]:.3f} rad/s, "
        #                   f"Torque = {status[3]:.3f} Nm")
                
        #         # Also read mechanical position parameter
        #         mech_pos = motor.read_single_param("mechPos")
        #         if mech_pos is not None:
        #             print(f"  Mechanical Position: {mech_pos:.3f} rad")
                
        #         time.sleep(1)
            
        #     motor.disable()
        #     print("✓ Motor disabled")
            
        # except Exception as e:
        #     print(f"✗ Error in position testing: {e}")
        #     try:
        #         motor.disable()
        #     except:
        #         pass

        # print("\n=== Testing All 4 Modes ===")

        
        # # Test 3: SPEED_MODE (速度模式)
        # print("\n3. Testing SPEED_MODE (速度模式)...")
        # try:
        #     motor.write_single_param("run_mode", motor.RunModes.SPEED_MODE.value)
        #     print("✓ Set to SPEED_MODE")
            
        #     motor.enable()
        #     print("✓ Motor enabled")
            
        #     # Test different speeds
        #     speeds = [3.0, -2.0, 5.0, 0.0]
        #     for speed in speeds:
        #         print(f"Setting speed: {speed} rad/s")
        #         motor.write_single_param("spd_ref", speed)
        #         time.sleep(3)  # Run at this speed for 3 seconds
                
        #         # Read current velocity parameters
        #         mech_vel = motor.read_single_param("mechVel")
        #         vbus = motor.read_single_param("VBUS")
        #         if mech_vel is not None:
        #             print(f"  Mechanical Velocity: {mech_vel:.3f} rad/s")
        #         if vbus is not None:
        #             print(f"  Bus Voltage: {vbus:.1f} V")
            
        #     motor.disable()
        #     print("✓ Motor disabled")
            
        # except Exception as e:
        #     print(f"✗ Error in SPEED_MODE: {e}")



        # # Test 1: CONTROL_MODE (运控模式)
        # print("\n1. Testing CONTROL_MODE (运控模式)...")
        # try:
        #     motor.write_single_param("run_mode", motor.RunModes.CONTROL_MODE.value)
        #     print("✓ Set to CONTROL_MODE")
            
        #     motor.enable()
        #     print("✓ Motor enabled")
            
        #     # Send control commands
        #     print("Sending control commands...")
        #     for i in range(5):
        #         torque = 0.5 * (1 if i % 2 == 0 else -1)  # Alternate torque
        #         response = motor.send_motor_control_command(
        #             torque=torque,
        #             target_angle=1.0,
        #             target_velocity=2.0,
        #             Kp=10,
        #             Kd=0.5
        #         )
        #         if response and response[0] is not None:
        #             print(f"  Command {i+1}: Pos={response[1]:.3f}rad, Vel={response[2]:.3f}rad/s")
        #         time.sleep(0.5)
                
        #     motor.disable()
        #     print("✓ Motor disabled")
            
        # except Exception as e:
        #     print(f"✗ Error in CONTROL_MODE: {e}")


        # # Test 4: CURRENT_MODE (电流模式)
        # print("\n4. Testing CURRENT_MODE (电流模式)...")
        # try:
        #     motor.write_single_param("run_mode", motor.RunModes.CURRENT_MODE.value)
        #     print("✓ Set to CURRENT_MODE")
            
        #     motor.enable()
        #     print("✓ Motor enabled")
            
        #     # Test different current/torque values
        #     currents = [0.3, -0.2, 0.5, 0.0]
        #     for current in currents:
        #         print(f"Setting current/torque: {current} A/Nm")
        #         motor.write_single_param("iq_ref", current)
        #         time.sleep(2)  # Apply current for 2 seconds
                
        #         # Read current parameters
        #         iqf = motor.read_single_param("iqf")
        #         rotation = motor.read_single_param("rotation")
        #         if iqf is not None:
        #             print(f"  iq filter value: {iqf:.3f} A")
        #         if rotation is not None:
        #             print(f"  Rotation turns: {rotation}")
            
        #     motor.disable()
        #     print("✓ Motor disabled")
            
        # except Exception as e:
        #     print(f"✗ Error in CURRENT_MODE: {e}")


        # # Test 2: POSITION_MODE (位置模式)
        # print("\n2. Testing POSITION_MODE (位置模式)...")
        # try:
        #     motor.write_single_param("run_mode", motor.RunModes.POSITION_MODE.value)
        #     print("✓ Set to POSITION_MODE")
            
        #     # Set position control parameters
        #     motor.write_single_param("limit_spd", 5.0)  # Limit speed to 5 rad/s
        #     motor.write_single_param("loc_kp", 20.0)    # Position gain
            
        #     motor.enable()
        #     print("✓ Motor enabled")
            
        #     # Move to different positions
        #     positions = [1.0, -1.0, 0.5, 0.0]
        #     for pos in positions:
        #         print(f"Moving to position: {pos} rad")
        #         motor.write_single_param("loc_ref", pos)
        #         time.sleep(2)  # Wait for movement
                
        #         # Read current position parameters
        #         mech_pos = motor.read_single_param("mechPos")
        #         run_mode = motor.read_single_param("run_mode")
        #         if mech_pos is not None:
        #             print(f"  Mechanical Position: {mech_pos:.3f} rad")
        #         if run_mode is not None:
        #             print(f"  Run Mode: {run_mode}")
            
        #     motor.disable()
        #     print("✓ Motor disabled")
            
        # except Exception as e:
        #     print(f"✗ Error in POSITION_MODE: {e}")



        # print("\n=== Final Parameter Reading ===")
        # print("Reading parameters after all tests...")
        # for param_name in parameters_to_test:
        #     try:
        #         value = motor.read_single_param(param_name)
        #         if value is not None:
        #             print(f"  {param_name}: {value}")
        #         else:
        #             print(f"  {param_name}: Failed to read")
        #     except Exception as e:
        #         print(f"  {param_name}: Error - {e}")

        # print("\n=== All Tests Completed ===")

    except can.CanError as e:
        print(f"CAN error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            bus.shutdown()
            print("CAN bus shutdown")
        except:
            pass