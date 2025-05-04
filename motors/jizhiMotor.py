import serial
import re

def main():
    # 配置串口参数
    port = 'COM13'
    baudrate = 19200

    # 正则表达式验证命令格式
    command_re = re.compile(r'^([mb][-+]?\d+\.\d+|[rpszxec])$')

    try:
        # 初始化串口连接
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print(f"Connected to {port} at {baudrate} baudrate.")
            
            while True:
                # 获取用户输入
                user_input = input("Enter command: ").strip().lower()
                
                if not user_input:
                    continue
                
                # 验证命令格式
                # if command_re.match(user_input):
                # 构造完整命令（自动添加\n\r结尾）
                full_command = f"{user_input}\r\ne"
                ser.write(full_command.encode('ascii'))
                print(f"Sent: {user_input}")
                
                # 读取并打印返回信息
                response = ser.readline().decode('ascii').strip()
                if response:
                    print(f"Received: {response}")
                else:
                    print("No response received.")
                # else:
                #     print("Invalid command! Valid formats:")
                #     print("- m/b followed by float (e.g. m10.0, b-5.5)")
                #     print("- Single letter commands: r, p, s, z, x, e, c")
    
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user.")

if __name__ == "__main__":
    main()
