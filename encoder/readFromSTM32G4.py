# import serial
# ser = serial.Serial('COM23', 115200)  # 根据实际端口修改
# while True:
#     line = ser.readline().decode().strip()
#     position = float(line)
#     print(f"Position: {position:.3f} rad")

# import serial
# import time
# from datetime import datetime

# # 配置串口参数
# port_name = 'COM23'
# baud_rate = 115200

# try:
#     # 打开串口
#     ser = serial.Serial(
#         port=port_name,
#         baudrate=baud_rate,
#         bytesize=serial.EIGHTBITS,
#         parity=serial.PARITY_NONE,
#         stopbits=serial.STOPBITS_ONE,
#         timeout=1  # 读取超时时间（秒）
#     )
    
#     print(f"开始监听串口 {port_name}，波特率 {baud_rate}...")
#     print("按 Ctrl+C 停止程序")
    
#     while True:
#         # 读取串口数据
#         data = ser.read_all()
        
#         if data:
#             # 获取当前时间
#             # current_time = datetime.now().strftime('[%Y-%m-%d %H:%M:%S.%f]')[:-3]
            
#             # # 将二进制数据转换为十六进制字符串
#             # hex_data = ' '.join([f"{byte:02X}" for byte in data])
            
#             # # 打印接收到的数据
#             # print(f"{current_time}# RECV HEX>")
#             # print(hex_data)

#             # 每行显示16个字节
#             hex_lines = []
#             for i in range(0, len(data), 16):
#                 chunk = data[i:i+16]
#                 hex_lines.append(' '.join([f"{byte:02X}" for byte in chunk]))

#             print('\n'.join(hex_lines))

        
#         # 短暂休眠以减少CPU占用
#         time.sleep(0.01)

# except serial.SerialException as e:
#     print(f"串口错误: {e}")
# except KeyboardInterrupt:
#     print("\n程序已停止")
# finally:
#     # 确保关闭串口
#     if 'ser' in locals() and ser.is_open:
#         ser.close()
#         print("串口已关闭")



import serial
import time
from datetime import datetime
import struct

class EncoderProcessor:
    def __init__(self):
        self.last_result = 0
        self.theta = 0
        self.init_count = 2  # 初始化计数器
        self.first_read = True
    
    def process(self, raw_value):
        """处理原始编码器值，返回累加角度"""
        # 初始阶段处理
        if self.init_count > 0:
            self.init_count -= 1
            self.theta = self.last_result = raw_value
            
            # 第二次读取后的特殊处理
            if self.init_count == 0 and self.theta > 32767:
                self.theta -= 65535
            return self.theta
        
        # 计算角度变化量
        temp = raw_value - self.last_result
        
        # 处理角度跨越65535/0边界的情况
        if temp > 43690:        # 正向跨越边界
            self.theta += temp - 65535
        elif temp < -43690:     # 反向跨越边界
            self.theta += temp + 65535
        else:                   # 正常变化
            self.theta += temp
        
        self.last_result = raw_value
        return self.theta

def main():
    # 配置串口参数
    port_name = 'COM23'
    baud_rate = 115200
    
    # 创建编码器处理器
    encoder = EncoderProcessor()
    
    try:
        # 打开串口
        ser = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1  # 100ms超时
        )
        
        print(f"开始监听串口 {port_name}，波特率 {baud_rate}...")
        print("按 Ctrl+C 停止程序")
        
        buffer = bytearray()
        last_print_time = time.time()
        
        while True:
            # 读取串口数据
            data = ser.read(ser.in_waiting or 1)
            if data:
                buffer.extend(data)
            
            # 处理完整的数据帧（每2字节为一组）
            while len(buffer) >= 2:
                # 提取并移除前2字节
                print(buffer)
                raw_bytes = buffer[:2]
                del buffer[:2]
                
                # 将字节转换为无符号短整型 (小端序)
                raw_value = struct.unpack('<H', raw_bytes)[0]
                
                # 处理编码器数据
                angle = encoder.process(raw_value)
                
                # 每秒打印10次角度值
                current_time = time.time()
                if current_time - last_print_time >= 0.1:
                    print(f"角度值: {angle:8.3f} (原始值: 0x{raw_value:04X} = {raw_value:5})")
                    last_print_time = current_time
    
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n程序已停止")
    finally:
        # 确保关闭串口
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")

if __name__ == "__main__":
    main()