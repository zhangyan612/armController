import struct
import can
import time
from typing import Optional, Callable

# Constants from robstride.h
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

# Control modes
Set_mode = 'j'
Set_parameter = 'p'
move_control_mode = 0
Pos_control_mode = 1
Speed_control_mode = 2
Elect_control_mode = 3
Set_Zero_mode = 4

# Communication types
Communication_Type_Get_ID = 0x00
Communication_Type_MotionControl = 0x01
Communication_Type_MotorRequest = 0x02
Communication_Type_MotorEnable = 0x03
Communication_Type_MotorStop = 0x04
Communication_Type_SetPosZero = 0x06
Communication_Type_Can_ID = 0x07
Communication_Type_Control_Mode = 0x12
Communication_Type_GetSingleParameter = 0x11
Communication_Type_SetSingleParameter = 0x12
Communication_Type_ErrorFeedback = 0x15

# Parameter indices
Index_List = [0x7005, 0x7006, 0x700A, 0x700B, 0x7010, 0x7011, 0x7014, 0x7016, 0x7017, 0x7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D]

class DataReadWriteOne:
    def __init__(self):
        self.index = 0
        self.data = 0.0

class DataReadWrite:
    def __init__(self, index_list=Index_List):
        self.run_mode = DataReadWriteOne()
        self.iq_ref = DataReadWriteOne()
        self.spd_ref = DataReadWriteOne()
        self.imit_torque = DataReadWriteOne()
        self.cur_kp = DataReadWriteOne()
        self.cur_ki = DataReadWriteOne()
        self.cur_filt_gain = DataReadWriteOne()
        self.loc_ref = DataReadWriteOne()
        self.limit_spd = DataReadWriteOne()
        self.limit_cur = DataReadWriteOne()
        self.mechPos = DataReadWriteOne()
        self.iqf = DataReadWriteOne()
        self.mechVel = DataReadWriteOne()
        self.VBUS = DataReadWriteOne()
        self.rotation = DataReadWriteOne()
        
        # Set indices
        self.run_mode.index = index_list[0]
        self.iq_ref.index = index_list[1]
        self.spd_ref.index = index_list[2]
        self.imit_torque.index = index_list[3]
        self.cur_kp.index = index_list[4]
        self.cur_ki.index = index_list[5]
        self.cur_filt_gain.index = index_list[6]
        self.loc_ref.index = index_list[7]
        self.limit_spd.index = index_list[8]
        self.limit_cur.index = index_list[9]
        self.mechPos.index = index_list[10]
        self.iqf.index = index_list[11]
        self.mechVel.index = index_list[12]
        self.VBUS.index = index_list[13]
        self.rotation.index = index_list[14]

class MotorPosRobStrideInfo:
    def __init__(self):
        self.Angle = 0.0
        self.Speed = 0.0
        self.Torque = 0.0
        self.Temp = 0.0
        self.pattern = 0

class MotorSet:
    def __init__(self):
        self.set_motor_mode = 0
        self.set_current = 0.0
        self.set_speed = 0.0
        self.set_Torque = 0.0
        self.set_angle = 0.0
        self.set_limit_cur = 0.0
        self.set_Kp = 0.0
        self.set_Ki = 0.0
        self.set_Kd = 0.0

# Math functions
def uint16_to_float(x, x_min, x_max, bits):
    span = (1 << bits) - 1
    offset = x_max - x_min
    return offset * x / span + x_min

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    offset = x_min
    if x > x_max:
        x = x_max
    elif x < x_min:
        x = x_min
    return int((x - offset) * ((1 << bits) - 1) / span)

def bytes_to_float(byte_data):
    """Convert 4 bytes to float (little endian)"""
    return struct.unpack('<f', byte_data)[0]

class RobStrideMotor:
    def __init__(self, can_id: int, bus: can.Bus, offset_func: Optional[Callable[[float], float]] = None):
        self.CAN_ID = can_id
        self.Master_CAN_ID = 0x00
        self.bus = bus
        self.Motor_Offset_MotoFunc = offset_func
        
        self.output = 0.0
        self.Can_Motor = 0
        self.Pos_Info = MotorPosRobStrideInfo()
        self.drw = DataReadWrite()
        self.error_code = 0
        self.Motor_Set_All = MotorSet()
        self.Motor_Set_All.set_motor_mode = move_control_mode

    def robstride_motor_analysis(self, data_frame: bytes, ext_id: int):
        """Parse incoming CAN messages"""
        can_id_from_msg = (ext_id & 0xFF00) >> 8
        
        if can_id_from_msg == self.CAN_ID:
            comm_type = (ext_id & 0x3F000000) >> 24
            
            if comm_type == 2:  # Motor feedback
                self.Pos_Info.Angle = uint16_to_float((data_frame[0] << 8) | data_frame[1], P_MIN, P_MAX, 16)
                self.Pos_Info.Speed = uint16_to_float((data_frame[2] << 8) | data_frame[3], V_MIN, V_MAX, 16)
                self.Pos_Info.Torque = uint16_to_float((data_frame[4] << 8) | data_frame[5], T_MIN, T_MAX, 16)
                self.Pos_Info.Temp = ((data_frame[6] << 8) | data_frame[7]) * 0.1
                self.error_code = (ext_id & 0x3F0000) >> 16
                self.Pos_Info.pattern = (ext_id & 0xC00000) >> 22
                
                print(f"Motor {self.CAN_ID:02X} - Angle: {self.Pos_Info.Angle:.3f}, "
                      f"Speed: {self.Pos_Info.Speed:.3f}, Torque: {self.Pos_Info.Torque:.3f}, "
                      f"Temp: {self.Pos_Info.Temp:.1f}°C")
                
            elif comm_type == 17:  # Parameter response
                index = (data_frame[1] << 8) | data_frame[0]
                try:
                    idx = Index_List.index(index)
                    value_bytes = data_frame[4:8]
                    
                    if idx == 0:
                        self.drw.run_mode.data = data_frame[4]
                    elif idx <= 13:  # Float parameters
                        float_value = bytes_to_float(value_bytes)
                        
                        if idx == 1:
                            self.drw.iq_ref.data = float_value
                        elif idx == 2:
                            self.drw.spd_ref.data = float_value
                        elif idx == 3:
                            self.drw.imit_torque.data = float_value
                        elif idx == 4:
                            self.drw.cur_kp.data = float_value
                        elif idx == 5:
                            self.drw.cur_ki.data = float_value
                        elif idx == 6:
                            self.drw.cur_filt_gain.data = float_value
                        elif idx == 7:
                            self.drw.loc_ref.data = float_value
                        elif idx == 8:
                            self.drw.limit_spd.data = float_value
                        elif idx == 9:
                            self.drw.limit_cur.data = float_value
                        elif idx == 10:
                            self.drw.mechPos.data = float_value
                        elif idx == 11:
                            self.drw.iqf.data = float_value
                        elif idx == 12:
                            self.drw.mechVel.data = float_value
                        elif idx == 13:
                            self.drw.VBUS.data = float_value
                            
                    print(f"Motor {self.CAN_ID:02X} - Parameter {index:04X}: {float_value}")
                    
                except ValueError:
                    pass
                    
            elif (ext_id & 0xFF) == 0xFE:  # CAN ID change
                self.CAN_ID = (ext_id & 0xFF00) >> 8

    def _send_can_message(self, ext_id: int, data: bytes):
        """Send CAN message with extended ID"""
        try:
            msg = can.Message(
                arbitration_id=ext_id,
                data=data,
                is_extended_id=True
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"Error sending CAN message: {e}")
            return False

    def robstride_get_can_id(self):
        """Get CAN ID from motor"""
        data = bytes([0x00] * 8)
        ext_id = Communication_Type_Get_ID << 24 | self.Master_CAN_ID << 8 | self.CAN_ID
        return self._send_can_message(ext_id, data)

    def robstride_motor_move_control(self, torque: float, angle: float, speed: float, kp: float, kd: float):
        """Send motion control command"""
        self.Motor_Set_All.set_Torque = torque
        self.Motor_Set_All.set_angle = angle
        self.Motor_Set_All.set_speed = speed
        self.Motor_Set_All.set_Kp = kp
        self.Motor_Set_All.set_Kd = kd
        
        # Change mode if needed
        if self.drw.run_mode.data != 0 and self.Pos_Info.pattern == 2:
            self.set_robstride_motor_parameter(0x7005, move_control_mode, Set_mode)
            self.get_robstride_motor_parameter(0x7005)
            self.Motor_Set_All.set_motor_mode = move_control_mode

        # Prepare CAN message
        ext_id = Communication_Type_MotionControl << 24 | float_to_uint(torque, T_MIN, T_MAX, 16) << 8 | self.CAN_ID
        
        data = bytes([
            float_to_uint(angle, P_MIN, P_MAX, 16) >> 8,
            float_to_uint(angle, P_MIN, P_MAX, 16) & 0xFF,
            float_to_uint(speed, V_MIN, V_MAX, 16) >> 8,
            float_to_uint(speed, V_MIN, V_MAX, 16) & 0xFF,
            float_to_uint(kp, KP_MIN, KP_MAX, 16) >> 8,
            float_to_uint(kp, KP_MIN, KP_MAX, 16) & 0xFF,
            float_to_uint(kd, KD_MIN, KD_MAX, 16) >> 8,
            float_to_uint(kd, KD_MIN, KD_MAX, 16) & 0xFF
        ])
        
        return self._send_can_message(ext_id, data)

    def enable_motor(self):
        """Enable motor"""
        data = bytes([0x00] * 8)
        ext_id = Communication_Type_MotorEnable << 24 | self.Master_CAN_ID << 8 | self.CAN_ID
        return self._send_can_message(ext_id, data)

    def disenable_motor(self, clear_error: int = 0):
        """Disable motor"""
        data = bytes([clear_error] + [0x00] * 7)
        ext_id = Communication_Type_MotorStop << 24 | self.Master_CAN_ID << 8 | self.CAN_ID
        return self._send_can_message(ext_id, data)

    def set_robstride_motor_parameter(self, index: int, value: float, value_mode: str):
        """Set motor parameter"""
        ext_id = Communication_Type_SetSingleParameter << 24 | self.Master_CAN_ID << 8 | self.CAN_ID
        
        if value_mode == 'p':  # Float parameter
            value_bytes = struct.pack('<f', value)
            data = bytes([
                index & 0xFF,
                index >> 8,
                0x00,
                0x00
            ]) + value_bytes
        elif value_mode == 'j':  # Mode parameter
            self.Motor_Set_All.set_motor_mode = int(value)
            data = bytes([
                index & 0xFF,
                index >> 8,
                0x00,
                0x00,
                int(value) & 0xFF,
                0x00,
                0x00,
                0x00
            ])
        
        return self._send_can_message(ext_id, data)

    def get_robstride_motor_parameter(self, index: int):
        """Get motor parameter"""
        data = bytes([
            index & 0xFF,
            index >> 8,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00
        ])
        ext_id = Communication_Type_GetSingleParameter << 24 | self.Master_CAN_ID << 8 | self.CAN_ID
        return self._send_can_message(ext_id, data)

    def set_zero_pos(self):
        """Set zero position"""
        self.disenable_motor(0)
        
        # Send set zero command
        data = bytes([0x01] + [0x00] * 7)
        ext_id = Communication_Type_SetPosZero << 24 | self.Master_CAN_ID << 8 | self.CAN_ID
        success = self._send_can_message(ext_id, data)
        
        # Re-enable motor
        if success:
            time.sleep(0.1)
            self.enable_motor()
        
        return success

# Test function to demonstrate usage
def test_motor_communication():
    """Test function to communicate with RobStride motor"""
    try:
        # Initialize CAN bus (adjust interface as needed)
        bus = can.Bus(interface='socketcan', channel='can0', bitrate=1000000)
        print("CAN bus initialized successfully")
        
        # Create motor instance (using CAN ID 0x01 as example)
        motor = RobStrideMotor(can_id=0x01, bus=bus)
        
        print("Testing motor communication...")
        
        # Test 1: Enable motor
        print("1. Enabling motor...")
        if motor.enable_motor():
            print("   Enable command sent")
        time.sleep(0.5)
        
        # Test 2: Get parameter (position)
        print("2. Requesting position parameter...")
        if motor.get_robstride_motor_parameter(0x7019):  # mechPos index
            print("   Position request sent")
        time.sleep(0.5)
        
        # Test 3: Read incoming messages for 2 seconds
        print("3. Listening for motor responses (5 seconds)...")
        start_time = time.time()
        while time.time() - start_time < 5:
            msg = bus.recv(timeout=1.0)
            if msg is not None:
                motor.robstride_motor_analysis(msg.data, msg.arbitration_id)
        
        # Test 4: Send zero torque command
        print("4. Sending zero torque command...")
        if motor.robstride_motor_move_control(torque=0.0, angle=0.0, speed=0.0, kp=0.0, kd=0.0):
            print("   Zero command sent")
        
        # Test 5: Disable motor
        print("5. Disabling motor...")
        if motor.disenable_motor():
            print("   Disable command sent")
        
        print("\nTest completed!")
        
    except Exception as e:
        print(f"Error during test: {e}")
        print("Make sure:")
        print("1. CAN interface is properly set up")
        print("2. You have permissions to access CAN interface")
        print("3. Motors are connected and powered")

if __name__ == "__main__":
    # Run the test
    test_motor_communication()