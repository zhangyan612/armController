import minimalmodbus
import serial
import time
import struct

class MotorController:
    def __init__(self, port, slave_address, baudrate=19200):
        """初始化MODBUS RTU连接"""
        self.instrument = minimalmodbus.Instrument(port, slave_address)
        self.instrument.serial.baudrate = baudrate
        self.instrument.serial.bytesize = 8
        self.instrument.serial.parity = serial.PARITY_NONE
        self.instrument.serial.stopbits = 1
        self.instrument.mode = minimalmodbus.MODE_RTU
        self.instrument.clear_buffers_before_each_transaction = True
        self.instrument.close_port_after_each_call = False
        self.instrument.serial.timeout = 1.0  # Increase timeout to 1 second
        
        # Debug output
        print(f"Serial port settings: {self.instrument.serial}")
        
    def read_register(self, address):
        """读取单个寄存器数据"""
        # For debugging, print the raw request
        self.instrument.debug = True
        result = self.instrument.read_register(address, functioncode=3)
        self.instrument.debug = False
        return result
    
    def write_register(self, address, value):
        """写入单个寄存器数据"""
        self.instrument.debug = True
        self.instrument.write_register(address, value, functioncode=6)
        self.instrument.debug = False
        
    def read_long(self, address):
        """读取32位长整数"""
        # From the screenshot, we can see the protocol uses 4 bytes (32-bit)
        self.instrument.debug = True
        # Reading 2 registers (4 bytes) for a 32-bit value
        data = self.instrument.read_registers(address, 2, functioncode=3)
        self.instrument.debug = False
        # Based on the protocol, the data format appears to be big endian
        value = (data[0] << 16) | data[1]
        # Handle negative values
        if value & 0x80000000:
            value -= 0x100000000
        return value

    def write_long(self, address, value):
        """写入32位长整数"""
        # Handle negative values
        if value < 0:
            value += 0x100000000
        # Split into high and low words
        high_word = (value >> 16) & 0xFFFF
        low_word = value & 0xFFFF
        # Write both registers
        self.instrument.debug = True
        self.instrument.write_registers(address, [high_word, low_word], functioncode=16)
        self.instrument.debug = False
        
    def set_servo_status(self, status):
        """设置伺服状态（开/关）"""
        try:
            print(f"Setting servo status to {status}")
            self.write_register(0x10, status)  # 0x10 = decimal 16
            time.sleep(0.2)
            return True
        except Exception as e:
            print(f"Error setting servo status: {e}")
            return False

    def get_servo_status(self):
        """获取伺服状态"""
        try:
            status = self.read_register(0x10)
            print(f"Current servo status: {status}")
            return status
        except Exception as e:
            print(f"Error reading servo status: {e}")
            return None

    def read_encoder_position(self):
        """读取编码器位置"""
        try:
            # Using register 0x15 (21 decimal) for encoder position 2
            position = self.read_register(0x15)
            print(f"Encoder position: {position}")
            return position
        except Exception as e:
            print(f"Error reading encoder position: {e}")
            return None

    def set_target_position(self, position):
        """设置目标位置"""
        try:
            print(f"Setting target position to {position}")
            # Using register 0x82 (130 decimal) for positioning with acceleration/deceleration
            self.write_long(0x82, position)
            return True
        except Exception as e:
            print(f"Error setting target position: {e}")
            return False

# 测试连接并诊断问题
def test_connection(port, addresses=None):
    """测试不同地址的连接，帮助诊断问题"""
    if addresses is None:
        addresses = [1, 2, 3]  # 尝试常见的设备地址
    
    print(f"Testing connection on port {port}")
    
    for address in addresses:
        try:
            # 创建基本的Modbus实例用于测试
            instrument = minimalmodbus.Instrument(port, address)
            instrument.serial.baudrate = 19200
            instrument.serial.bytesize = 8
            instrument.serial.parity = serial.PARITY_NONE
            instrument.serial.stopbits = 1
            instrument.mode = minimalmodbus.MODE_RTU
            instrument.serial.timeout = 1.0
            
            print(f"Testing with address: {address}")
            
            # 尝试简单的读取操作
            instrument.debug = True
            try:
                # 尝试读取伺服状态寄存器
                result = instrument.read_register(0x10, functioncode=3)
                print(f"Success! Address {address} responded with value {result}")
                return address
            except Exception as e:
                print(f"Failed with address {address}: {e}")
                
        except Exception as e:
            print(f"Error setting up instrument with address {address}: {e}")
    
    print("Connection test failed with all addresses")
    return None

# 直接发送原始Modbus RTU命令测试
def send_raw_command(port, command_bytes):
    """发送原始命令并接收响应，用于调试"""
    try:
        # 设置串口
        ser = serial.Serial(
            port=port,
            baudrate=19200,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=1,
            timeout=1
        )
        
        # 确保串口已打开
        if not ser.is_open:
            ser.open()
            
        # 清空缓冲区
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # 发送命令
        print(f"Sending raw bytes: {' '.join([f'{b:02X}' for b in command_bytes])}")
        ser.write(command_bytes)
        
        # 给设备响应的时间
        time.sleep(0.1)
        
        # 检查是否有数据可以读取
        if ser.in_waiting > 0:
            # 读取响应
            response = ser.read(ser.in_waiting)
            print(f"Received: {' '.join([f'{b:02X}' for b in response])}")
            return response
        else:
            print("No response received")
            return None
            
    except Exception as e:
        print(f"Error in raw command: {e}")
        return None
    finally:
        # 关闭串口
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    port = 'COM8'
    
    # 首先测试基本连接
    print("\n===== 基本连接测试 =====")
    working_address = test_connection(port)
    
    if working_address is None:
        print("\n===== 尝试直接发送原始命令 =====")
        # 从截图中看到的查询命令: 01 03 00 13 00 00 00 04 C7 35
        # 尝试读取伺服状态命令: 01 03 00 10 00 00 00 02 C5 F2
        command = bytes([0x01, 0x03, 0x00, 0x10, 0x00, 0x00, 0x00, 0x02, 0xC5, 0xF2])
        response = send_raw_command(port, command)
        
        # 再尝试一个命令 - 读取编码器位置
        command = bytes([0x01, 0x03, 0x00, 0x15, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x32])
        response = send_raw_command(port, command)
        
        print("\nVerifying port access and settings:")
        try:
            # 尝试直接打开串口检查设置
            ser = serial.Serial(port, baudrate=19200, timeout=1)
            print(f"Port {port} opened successfully.")
            print(f"Port settings: {ser}")
            ser.close()
        except Exception as e:
            print(f"Error opening port directly: {e}")
        
        print("\n无法建立Modbus连接。请检查:")
        print("1. 确保设备已连接并通电")
        print("2. 验证COM端口号是否正确")
        print("3. 检查波特率设置 (19200)")
        print("4. 确保设备地址正确 (01)")
        print("5. 检查是否有其他程序占用端口")
    else:
        print(f"\n连接成功! 使用地址: {working_address}")
        
        try:
            # 尝试使用找到的地址创建控制器
            mc = MotorController(port=port, slave_address=working_address)
            
            # 读取伺服状态
            servo_status = mc.get_servo_status()
            
            if servo_status is not None:
                # 尝试启动伺服
                if mc.set_servo_status(1):
                    print("伺服已启动")
                    
                    # 读取当前位置
                    current_pos = mc.read_encoder_position()
                    
                    if current_pos is not None:
                        # 移动到新位置
                        new_pos = (current_pos + 100) % 32767  # 增加100步但保持在范围内
                        if mc.set_target_position(new_pos):
                            print(f"正在移动到位置 {new_pos}")
                            time.sleep(3)  # 等待移动完成
                            
                            # 读取新位置
                            new_current_pos = mc.read_encoder_position()
                            print(f"移动后位置: {new_current_pos}")
                    
                    # 关闭伺服
                    mc.set_servo_status(0)
                    print("伺服已关闭")
            
        except Exception as e:
            print(f"操作过程中发生错误: {e}")