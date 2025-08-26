import serial
import time
import binascii
import struct

SERIAL_PORT = 'COM21'
BAUDRATE = 921600

class MotorController:
    def __init__(self, ser_connection, motor_id, motor_type):
        self.ser = ser_connection
        self.motor_id = motor_id
        self.motor_type = motor_type  # 'left' 或 'right'
        
        # 根据电机类型计算节点ID
        if self.motor_type == 'left':
            self.node_id = motor_id + 0x0B   # 左电机计算方式
        else:  # right motor
            self.node_id = (motor_id << 2) + 0x08  # 右电机计算方式
            
        self.initialize_motor()
    
    def initialize_motor(self):
        print(f"Initializing {self.motor_type} motor ID {self.motor_id}...")
        init_cmd1 = bytes.fromhex(f"41 54 00 07 e8 {self.motor_id:02x} 02 01 00 0d 0a")
        self.ser.write(init_cmd1)
        print(f"Sent to ID {self.motor_id}: {init_cmd1.hex(' ')}")
        time.sleep(0.1)
        
        init_cmd2 = bytes.fromhex(f"41 54 20 07 e8 {self.motor_id:02x} 08 00 c4 00 00 00 00 00 00 0d 0a")
        self.ser.write(init_cmd2)
        print(f"Sent to ID {self.motor_id}: {init_cmd2.hex(' ')}")
        time.sleep(0.1)

    def build_command(self, comm_type, data_bytes):
        """构建带长度字节的CAN帧"""
        ext_id_bytes = bytes([comm_type, 0x07, 0xE8, self.node_id])
        data_len = bytes([len(data_bytes)])
        frame = b"\x41\x54" + ext_id_bytes + data_len + data_bytes + b"\x0d\x0a"
        return frame
    
    def float_to_hex(self, f):
        """浮点数转小端十六进制"""
        return struct.pack('<f', f)
    
    def send_command(self, frame):
        """发送帧并读取响应"""
        self.ser.write(frame)
        hex_frame = frame.hex(' ')
        print(f"Sent to {self.motor_type} motor ID {self.motor_id}: {hex_frame}")
        time.sleep(0.05)
        response = self.ser.read_all()
        if response:
            hex_response = response.hex()
            print(f"Received: {hex_response}")
        return response
    
    def set_position_mode(self):
        """设置位置模式"""
        print(f"Setting position mode for {self.motor_type} motor {self.motor_id}")
        
        # 根据您提供的CAN指令，设置位置模式的命令
        if self.motor_id == 1:
            # CAN ID 1 的位置模式命令
            pos_mode_cmd = bytes.fromhex("41 54 90 07 e8 0c 08 05 70 00 00 05 00 00 00 0d 0a")
        else:  # motor_id == 2
            # CAN ID 2 的位置模式命令
            pos_mode_cmd = bytes.fromhex("41 54 90 07 e8 14 08 05 70 00 00 05 00 00 00 0d 0a")
            
        self.ser.write(pos_mode_cmd)
        print(f"Sent position mode command: {pos_mode_cmd.hex(' ')}")
        time.sleep(0.2)
    
    def enable_motor(self):
        """使能电机"""
        print(f"Enabling {self.motor_type} motor {self.motor_id}")
        
        # 根据您提供的CAN指令，使能电机的命令
        if self.motor_id == 1:
            # CAN ID 1 的使能命令
            enable_cmd = bytes.fromhex("41 54 18 07 e8 0c 08 00 00 00 00 00 00 00 00 0d 0a")
        else:  # motor_id == 2
            # CAN ID 2 的使能命令
            enable_cmd = bytes.fromhex("41 54 18 07 e8 14 08 00 00 00 00 00 00 00 00 0d 0a")
            
        self.ser.write(enable_cmd)
        print(f"Sent enable command: {enable_cmd.hex(' ')}")
        time.sleep(0.5)
    
    def disable_motor(self):
        """禁用电机"""
        print(f"Disabling {self.motor_type} motor {self.motor_id}")
        
        # 根据您提供的CAN指令，禁用电机的命令
        if self.motor_id == 1:
            # CAN ID 1 的禁用命令
            disable_cmd = bytes.fromhex("41 54 20 07 e8 0c 08 00 00 00 00 00 00 00 00 0d 0a")
        else:  # motor_id == 2
            # CAN ID 2 的禁用命令
            disable_cmd = bytes.fromhex("41 54 20 07 e8 14 08 00 00 00 00 00 00 00 00 0d 0a")
            
        self.ser.write(disable_cmd)
        print(f"Sent disable command: {disable_cmd.hex(' ')}")
        time.sleep(0.5)
    
    def move_to_position(self, position_rad):
        """移动到指定位置"""
        print(f"Moving {self.motor_type} motor {self.motor_id} to {position_rad} rad")
        
        # 设置速度 (1.0 rad/s)
        vel_data = bytes.fromhex("24 70 00 00") + self.float_to_hex(1.0)
        vel_frame = self.build_command(0x90, vel_data)
        self.send_command(vel_frame)
        time.sleep(0.05)
        
        # 设置加速度 (5.0 rad/s²)
        acc_data = bytes.fromhex("25 70 00 00") + self.float_to_hex(5.0)
        acc_frame = self.build_command(0x90, acc_data)
        self.send_command(acc_frame)
        time.sleep(0.05)
        
        # 设置位置
        pos_data = bytes.fromhex("16 70 00 00") + self.float_to_hex(position_rad)
        pos_frame = self.build_command(0x90, pos_data)
        self.send_command(pos_frame)
        time.sleep(0.1)

def main():
    print("Starting dual motor control...")
    
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.5
    )
    
    print("Initializing adapter...")
    ser.write(bytes.fromhex("41 54 2b 41 54 0d 0a"))
    time.sleep(0.2)
    
    # 创建两个电机控制器
    left_motor = MotorController(ser, motor_id=0x0D, motor_type='left')
    # right_motor = MotorController(ser, motor_id=0x01, motor_type='right')
    
    try:
        # 设置模式
        left_motor.set_position_mode()
        # right_motor.set_position_mode()
        
        # 使能电机
        left_motor.enable_motor()
        # right_motor.enable_motor()
        time.sleep(1)
        
        print("\n--- Moving motors together ---")
        
        # 同时移动两个电机
        left_motor.move_to_position(0.0)
        # right_motor.move_to_position(0.0)
        time.sleep(1)

        for i in range(3):
            left_motor.move_to_position(-0.5)
            time.sleep(1)
            
            left_motor.move_to_position(0.0)
            # right_motor.move_to_position(0.5)
            time.sleep(1)
            # right_motor.move_to_position(0.0)

        # 禁用电机
        left_motor.disable_motor()
        # right_motor.disable_motor()
        
        print("Dual motor control completed successfully.")
    
    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        ser.close()
        print("Serial connection closed")

if __name__ == "__main__":
    main()






# Enabling motor...
# Sent: 41 54 18 07 e8 14 08 00 00 00 00 00 00 00 00 0d 0a
# Received: 4154140017ec0880007fec7fff00dc0d0a

# 172 [2025/08/25 11:12:25]  -> COM21: 41 54 90 07 e8 14 08 05 70 00 00 01 00 00 00 0d 0a
# 173 [2025/08/25 11:12:25]  -> COM21: 41 54 18 07 e8 14 08 00 00 00 00 00 00 00 00 0d 0a

# Moving to position 0.0 with velocity 2.0 and acceleration 5.0...
# Setting velocity to 2.0...
# Sent: 41 54 90 07 e8 14 08 24 70 00 00 00 40 00 00 00 0d 0a
# Setting acceleration to 5.0...
# Sent: 41 54 90 07 e8 14 08 25 70 00 00 00 40 a0 00 00 0d 0a
# Setting position to 0.0...
# Sent: 41 54 90 07 e8 14 08 16 70 00 00 00 00 00 00 00 0d 0a
# Moving to position 2.0 with velocity 2.0 and acceleration 5.0...

# 177 [2025/08/25 11:13:31]  -> COM21: 41 54 90 07 e8 14 08 24 70 00 00 00 00 00 40 0d 0a
# 178 [2025/08/25 11:13:31]  -> COM21: 41 54 90 07 e8 14 08 25 70 00 00 00 00 a0 40 0d 0a
# 179 [2025/08/25 11:13:31]  -> COM21: 41 54 90 07 e8 14 08 16 70 00 00 00 00 00 40 0d 0a

# Setting velocity to 2.0...
# Sent: 41 54 90 07 e8 14 08 24 70 00 00 00 40 00 00 00 0d 0a
# Setting acceleration to 5.0...
# Sent: 41 54 90 07 e8 14 08 25 70 00 00 00 40 a0 00 00 0d 0a
# Setting position to 2.0...
# Sent: 41 54 90 07 e8 14 08 16 70 00 00 00 40 00 00 00 0d 0a
# Moving to position 0.0 with velocity 2.0 and acceleration 5.0...

# 174 [2025/08/25 11:12:54]  -> COM21: 41 54 90 07 e8 14 08 24 70 00 00 00 00 00 40 0d 0a
# 175 [2025/08/25 11:12:54]  -> COM21: 41 54 90 07 e8 14 08 25 70 00 00 00 00 a0 40 0d 0a
# 176 [2025/08/25 11:12:54]  -> COM21: 41 54 90 07 e8 14 08 16 70 00 00 00 00 00 00 0d 0a



# Disabling motor...
# Sent: 41 54 20 07 e8 14 08 a8 1c 3b 4e 84 11 b0 08 0d 0a
# Received: 4154100017ec087fff7fd6804700dc0d0a

# 180 [2025/08/25 11:13:51]  -> COM21: 41 54 20 07 e8 14 08 a8 1c 3b 4e 84 11 b0 08 0d 0a