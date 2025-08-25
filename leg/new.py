import serial
import time
import struct
from enum import Enum
from typing import Optional, Callable
import threading

class RobStrideMotor:
    """RobStride 电机控制类，指令与上位机完全一致"""
    
    def __init__(self, ser: serial.Serial, motor_id: int = 0x14):
        """
        初始化 RobStride 电机
        
        Args:
            ser: 串口对象
            motor_id: 电机ID (默认为0x14)
        """
        self.ser = ser
        self.motor_id = motor_id
        self.CAN_ID = motor_id  # 固定为0x14
    
    def send_command(self, frame_hex: str):
        """
        发送十六进制格式的帧
        
        Args:
            frame_hex: 十六进制字符串格式的帧
        """
        frame = bytes.fromhex(frame_hex.replace(" ", ""))
        self.ser.write(frame)
        print(f"Sent: {frame_hex}")
        time.sleep(0.05)
        
        # 读取响应
        response = self.ser.read_all()
        if response:
            hex_response = response.hex()
            print(f"Received: {hex_response}")
        return response
    
    def initialize_adapter(self):
        """初始化适配器"""
        print("Initializing adapter...")
        self.send_command("41 54 2b 41 54 0d 0a")
        time.sleep(0.2)
    
    def send_initialization_sequence(self):
        """发送初始化序列"""
        print("Sending initialization sequence...")
        
        # 发送初始化命令序列 (1-33)
        for i in range(1, 34):
            # 计算CAN ID部分
            base = 0xe8 if i <= 16 else (0xe9 if i <= 24 else (0xea if i <= 32 else 0xeb))
            offset = (i - 1) % 16 * 0x40
            can_id_byte = base + (offset // 0x100)
            can_id_low = offset % 0x100
            
            # 构建指令
            cmd = f"41 54 00 07 {can_id_byte:02x} {can_id_low:02x} 01 00 0d 0a"
            self.send_command(cmd)
            time.sleep(0.01)
    
    def setup_motor(self):
        """设置电机参数"""
        print("Setting up motor...")
        
        # 指令34
        self.send_command("41 54 00 07 e8 14 01 00 0d 0a")
        time.sleep(0.1)
        
        # 指令35 - 设置电机参数
        self.send_command("41 54 20 07 e8 14 08 00 c4 00 00 00 00 00 00 0d 0a")
        time.sleep(0.1)
    
    def set_position_mode(self):
        """设置位置模式"""
        print("Setting position mode...")
        # 指令37
        self.send_command("41 54 90 07 e8 14 08 05 70 00 00 01 00 00 00 0d 0a")
        time.sleep(0.2)
    
    def enable_motor(self):
        """使能电机"""
        print("Enabling motor...")
        # 指令38
        self.send_command("41 54 18 07 e8 14 08 00 00 00 00 00 00 00 00 0d 0a")
        time.sleep(0.5)
    
    def set_velocity(self, velocity: float = 2.0):
        """
        设置速度
        
        Args:
            velocity: 速度值 (默认2.0)
        """
        print(f"Setting velocity to {velocity}...")
        
        # 将速度值转换为十六进制（小端字节序）
        vel_bytes = struct.pack('<f', velocity)
        vel_hex = vel_bytes.hex()
        
        # 设置速度指令 - 与上位机完全一致
        # 上位机格式: 41 54 90 07 e8 14 08 24 70 00 00 00 00 00 40 0d 0a
        speed_cmd = f"41 54 90 07 e8 14 08 24 70 00 00 00 {vel_hex[6:8]} {vel_hex[4:6]} {vel_hex[2:4]} {vel_hex[0:2]} 0d 0a"
        self.send_command(speed_cmd)
        time.sleep(0.1)
    
    def set_acceleration(self, acceleration: float = 5.0):
        """
        设置加速度
        
        Args:
            acceleration: 加速度值 (默认5.0)
        """
        print(f"Setting acceleration to {acceleration}...")
        
        # 将加速度值转换为十六进制（小端字节序）
        acc_bytes = struct.pack('<f', acceleration)
        acc_hex = acc_bytes.hex()
        
        # 设置加速度指令 - 与上位机完全一致
        # 上位机格式: 41 54 90 07 e8 14 08 25 70 00 00 00 00 a0 40 0d 0a
        acc_cmd = f"41 54 90 07 e8 14 08 25 70 00 00 00 {acc_hex[6:8]} {acc_hex[4:6]} {acc_hex[2:4]} {acc_hex[0:2]} 0d 0a"
        self.send_command(acc_cmd)
        time.sleep(0.1)
    
    def set_position(self, position: float):
        """
        设置目标位置
        
        Args:
            position: 目标位置值
        """
        print(f"Setting position to {position}...")
        
        # 将位置值转换为十六进制（小端字节序）
        pos_bytes = struct.pack('<f', position)
        pos_hex = pos_bytes.hex()
        
        # 设置位置指令 - 与上位机完全一致
        # 上位机格式: 41 54 90 07 e8 14 08 16 70 00 00 00 00 00 00 0d 0a (位置0)
        # 上位机格式: 41 54 90 07 e8 14 08 16 70 00 00 00 00 00 40 0d 0a (位置2)
        pos_cmd = f"41 54 90 07 e8 14 08 16 70 00 00 00 {pos_hex[6:8]} {pos_hex[4:6]} {pos_hex[2:4]} {pos_hex[0:2]} 0d 0a"
        self.send_command(pos_cmd)
        time.sleep(0.1)
    
    def move_to_position(self, velocity: float, acceleration: float, position: float):
        """
        移动到指定位置
        
        Args:
            velocity: 速度值
            acceleration: 加速度值
            position: 目标位置值
        """
        print(f"Moving to position {position} with velocity {velocity} and acceleration {acceleration}...")
        
        # 设置速度
        self.set_velocity(velocity)
        
        # 设置加速度
        self.set_acceleration(acceleration)
        
        # 设置位置
        self.set_position(position)
    
    def disable_motor(self):
        """禁用电机"""
        print("Disabling motor...")
        # 指令48
        self.send_command("41 54 20 07 e8 14 08 a8 1c 3b 4e 84 11 b0 08 0d 0a")
        time.sleep(0.5)


def main():
    """主函数 - 执行完整的电机控制流程"""
    print("Starting complete motor control sequence...")
    
    # 初始化串口
    ser = serial.Serial(
        port='COM21',  # 请根据实际情况修改端口
        baudrate=921600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.5
    )
    
    # 创建电机对象
    motor = RobStrideMotor(ser)
    
    try:
        # 1. 初始化适配器
        motor.initialize_adapter()
        time.sleep(0.2)
        
        # 2. 发送初始化序列
        motor.send_initialization_sequence()
        time.sleep(0.2)
        
        # 3. 设置电机参数
        motor.setup_motor()
        time.sleep(0.2)
        
        # 4. 设置位置模式
        motor.set_position_mode()
        time.sleep(0.2)
        
        # 5. 使能电机
        motor.enable_motor()
        time.sleep(1)
        
        # 6. 移动到位置0（速度2，加速度5）
        motor.move_to_position(2.0, 5.0, 0.0)
        time.sleep(5)  # 等待电机移动
        
        # 7. 移动到位置2（速度2，加速度5）
        motor.move_to_position(2.0, 5.0, 2.0)
        time.sleep(5)  # 等待电机移动
        
        # 8. 移动到位置0（速度2，加速度5）
        motor.move_to_position(2.0, 5.0, 0.0)
        time.sleep(5)  # 等待电机移动
        
        # 9. 禁用电机
        motor.disable_motor()
        
        print("Complete motor control sequence finished successfully.")
    
    except Exception as e:
        print(f"Error: {str(e)}")
        import traceback
        traceback.print_exc()
    finally:
        ser.close()
        print("Serial connection closed.")


if __name__ == "__main__":
    main()