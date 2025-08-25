import serial
import time
import binascii

# 串口配置
SERIAL_PORT = 'COM21'
BAUDRATE = 921600

class SerialMotorController:
    def __init__(self, port, baudrate):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1.0
        )
        self.initialize_adapter()
    
    def initialize_adapter(self):
        """发送初始化序列 - 完全复制日志中的命令"""
        init_sequence = [
            "41 54 2b 41 54 0d 0a",
            "41 54 00 07 e8 0c 01 00 0d 0a",
            "41 54 00 07 e8 44 01 00 0d 0a",
            "41 54 00 07 e8 84 01 00 0d 0a",
            "41 54 00 07 e8 c4 01 00 0d 0a",
            "41 54 00 07 e9 04 01 00 0d 0a",
            "41 54 00 07 e9 44 01 00 0d 0a",
            "41 54 00 07 e9 84 01 00 0d 0a",
            "41 54 00 07 e9 c4 01 00 0d 0a",
            "41 54 00 07 ea 04 01 00 0d 0a",
            "41 54 00 07 ea 44 01 00 0d 0a",
            "41 54 00 07 ea 84 01 00 0d 0a",
            "41 54 00 07 ea c4 01 00 0d 0a",
            "41 54 00 07 eb 04 01 00 0d 0a",
            "41 54 00 07 eb 44 01 00 0d 0a",
            "41 54 00 07 eb 84 01 00 0d 0a",
            "41 54 00 07 eb c4 01 00 0d 0a",
            "41 54 00 07 e8 0c 01 00 0d 0a",
            "41 54 20 07 e8 0c 08 00 c4 00 00 00 00 00 00 0d 0a",
            "41 54 00 07 e8 4c 01 00 0d 0a",
        ]
        
        print("Sending initialization sequence...")
        for hex_str in init_sequence:
            self.send_hex_command(hex_str)
            time.sleep(0.05)
    
    def send_hex_command(self, hex_str):
        """发送十六进制字符串命令"""
        # 移除空格并转换为字节
        hex_clean = hex_str.replace(" ", "")
        data = bytes.fromhex(hex_clean)
        
        # 发送命令
        self.ser.write(data)
        print(f"Sent: {hex_str}")
        
        # 读取响应
        response = self.ser.read_all()
        if response:
            hex_response = binascii.hexlify(response).decode()
            print(f"Received: {hex_response}")
        return response
    
    def set_position_mode(self):
        """设置位置模式 (完全复制日志中的命令)"""
        print("Setting position mode...")
        self.send_hex_command("41 54 90 07 e8 0c 08 05 70 00 00 01 00 00 00 0d 0a")
    
    def enable_motor(self):
        """使能电机 (完全复制日志中的命令)"""
        print("Enabling motor...")
        self.send_hex_command("41 54 18 07 e8 0c 08 00 00 00 00 00 00 00 00 0d 0a")
    
    def move_to_0_5_rad(self):
        """移动到0.5 rad位置 (根据日志中0.5 rad的命令)"""
        print("Moving to 0.5 rad...")
        # 发送三个命令序列 (日志中的273-275行)
        self.send_hex_command("41 54 90 07 e8 0c 08 24 70 00 00 00 00 80 3f 0d 0a")
        self.send_hex_command("41 54 90 07 e8 0c 08 25 70 00 00 00 00 80 3f 0d 0a")
        self.send_hex_command("41 54 90 07 e8 0c 08 16 70 00 00 00 00 00 3f 0d 0a")
    
    def move_to_0_rad(self):
        """移动到0 rad位置 (完全复制日志中的命令)"""
        print("Moving to 0 rad...")
        # 发送三个命令序列 (日志中的124-126行)
        self.send_hex_command("41 54 90 07 e8 0c 08 24 70 00 00 00 00 20 41 0d 0a")
        self.send_hex_command("41 54 90 07 e8 0c 08 25 70 00 00 00 00 20 41 0d 0a")
        self.send_hex_command("41 54 90 07 e8 0c 08 16 70 00 00 00 00 00 00 0d 0a")
    
    def disable_motor(self):
        """禁用电机 (完全复制日志中的命令)"""
        print("Disabling motor...")
        self.send_hex_command("41 54 20 07 e8 0c 08 5a 41 30 02 0c 34 38 0a 0d 0a")
    
    def close(self):
        """关闭串口连接"""
        self.ser.close()

def main():
    print("Starting motor control...")
    controller = SerialMotorController(SERIAL_PORT, BAUDRATE)
    
    try:
        # 设置位置模式
        controller.set_position_mode()
        time.sleep(1)
        
        # 使能电机
        controller.enable_motor()
        time.sleep(1)
        
        # 移动到0位
        controller.move_to_0_rad()
        time.sleep(3)  # 等待移动完成
        
        # 移动到0.5 rad (修改点)
        controller.move_to_0_5_rad()
        time.sleep(3)  # 等待移动完成
        
        # 回到0位
        controller.move_to_0_rad()
        time.sleep(3)  # 等待移动完成
        
        # 禁用电机
        controller.disable_motor()
        
        print("Motor control completed successfully.")
    
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        controller.close()
        print("Connection closed")

if __name__ == "__main__":
    main()