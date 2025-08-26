import serial
import time
import binascii
import struct

SERIAL_PORT = 'COM37'
BAUDRATE = 921600

class MotorController:
    def __init__(self, ser_connection, motor_id=1, host_id=1):
        self.ser = ser_connection
        self.motor_id = motor_id
        self.host_id = host_id
        self.initialize_motor()
    
    def initialize_motor(self):
        print(f"Initializing motor ID {self.motor_id}...")
        # 初始化命令1 (带长度字节)
        init_cmd1 = bytes.fromhex(f"41 54 00 07 e8 {self.motor_id:02x} 02 01 00 0d 0a")
        self.ser.write(init_cmd1)
        print(f"Sent to ID {self.motor_id}: {init_cmd1.hex(' ')}")
        time.sleep(0.1)
        
        # 初始化命令2 (带长度字节)
        init_cmd2 = bytes.fromhex(f"41 54 20 07 e8 {self.motor_id:02x} 08 00 c4 00 00 00 00 00 00 0d 0a")
        self.ser.write(init_cmd2)
        print(f"Sent to ID {self.motor_id}: {init_cmd2.hex(' ')}")
        time.sleep(0.1)

    def build_command(self, comm_type, data_bytes):
        """构建带长度字节的CAN帧"""
        node_id = self.motor_id + 0x0B   # 协议映射修正
        ext_id_bytes = bytes([comm_type, 0x07, 0xE8, node_id])
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
        print(f"Sent to ID {self.motor_id}: {hex_frame}")
        time.sleep(0.05)
        response = self.ser.read_all()
        if response:
            hex_response = response.hex()
            print(f"Received: {hex_response}")
        return response
    
    def set_position_mode(self):
        """设置位置模式 (带长度字节)"""
        print(f"Setting position mode for motor {self.motor_id}")
        # 修正命令格式: 05 70 00 00 01 00 00 00
        data = bytes.fromhex("05 70 00 00 01 00 00 00")
        frame = self.build_command(0x90, data)
        self.send_command(frame)
        time.sleep(0.2)
    
    def enable_motor(self):
        """使能电机 (带长度字节)"""
        print(f"Enabling motor {self.motor_id}")
        data = bytes.fromhex("00 00 00 00 00 00 00 00")
        frame = self.build_command(0x18, data)
        self.send_command(frame)
        time.sleep(0.5)
    
    def move_to_position(self, position_rad):
        """移动到指定位置 (包含速度和加速度设置)"""
        print(f"Moving motor {self.motor_id} to {position_rad} rad")
        
        # 1. 设置速度 (1.0 rad/s)
        vel_data = bytes.fromhex("24 70 00 00") + self.float_to_hex(1.0)
        vel_frame = self.build_command(0x90, vel_data)
        self.send_command(vel_frame)
        time.sleep(0.05)
        
        # 2. 设置加速度 (5.0 rad/s²)
        acc_data = bytes.fromhex("25 70 00 00") + self.float_to_hex(5.0)
        acc_frame = self.build_command(0x90, acc_data)
        self.send_command(acc_frame)
        time.sleep(0.05)
        
        # 3. 设置位置 (修正索引顺序)
        pos_data = bytes.fromhex("16 70 00 00") + self.float_to_hex(position_rad)
        pos_frame = self.build_command(0x90, pos_data)
        self.send_command(pos_frame)
        time.sleep(0.1)
    
    def disable_motor(self):
        """禁用电机 (使用0x20通信类型)"""
        print(f"Disabling motor {self.motor_id}")
        # 修正为上位机格式: 01 00 00 00 00 00 00 00
        data = bytes.fromhex("01 00 00 00 00 00 00 00")
        frame = self.build_command(0x20, data)
        self.send_command(frame)

def main():
    print("Starting motor control...")
    
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
    
    #motor1 = MotorController(ser, motor_id=0x0C)
    motor2 = MotorController(ser, motor_id=0x0D)
    
    try:
        #motor1.set_position_mode()
        #motor1.enable_motor()
        
        motor2.set_position_mode()
        motor2.enable_motor()
        time.sleep(1)
        
        print("\n--- Moving motors ---")

        
        motor2.move_to_position(0.0)
        time.sleep(3)
        motor2.move_to_position(-0.5)
        time.sleep(3)
        motor2.move_to_position(0.0)
        time.sleep(3)
        
        #motor1.disable_motor()
        motor2.disable_motor()
        
        print("Motor control completed successfully.")
    
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        ser.close()
        print("Serial connection closed")

if __name__ == "__main__":
    main()