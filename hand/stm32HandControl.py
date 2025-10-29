import serial
import threading
import time
import sys
import os
import re

class MotorController:
    def __init__(self, port='COM3', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.running = False
        self.receive_thread = None
        self.silent_mode = True  # 静默模式，不显示状态信息
        self.last_command = None
        
    def connect(self):
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            
            if self.ser.is_open:
                print(f"成功连接到 {self.port}")
                self.running = True
                # 启动接收线程
                self.receive_thread = threading.Thread(target=self._receive_data)
                self.receive_thread.daemon = True
                self.receive_thread.start()
                return True
            else:
                print("连接失败")
                return False
                
        except Exception as e:
            print(f"连接错误: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("已断开连接")
    
    def _receive_data(self):
        """接收数据的线程函数"""
        buffer = ""
        while self.running:
            try:
                if self.ser and self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # 按行处理接收到的数据
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self._process_received_line(line)
                
                time.sleep(0.01)
                
            except Exception as e:
                if self.running:  # 只在运行状态下打印错误
                    print(f"接收错误: {e}")
                break
    
    def _process_received_line(self, line):
        """处理接收到的每一行数据"""
        # 如果是状态信息 (Pos: xxx | Target: xxx | ...)
        if line.startswith("Pos:") or line.startswith("Control:"):
            if not self.silent_mode:
                print(f"状态: {line}")
        # 如果是调试信息
        elif line.startswith("ADC Raw:") or line.startswith("CMD:") or line.startswith("Debug:"):
            if not self.silent_mode:
                print(f"调试: {line}")
        # 如果是命令响应
        elif any(keyword in line for keyword in ["Target set to", "System stopped", "System started", 
                                               "Motor test", "Invalid target", "Unknown command"]):
            print(f"响应: {line}")
        # 其他信息
        else:
            print(f"信息: {line}")
    
    def send_command(self, command):
        """发送命令"""
        if not self.ser or not self.ser.is_open:
            print("串口未连接")
            return False
            
        try:
            # 保存最后发送的命令
            self.last_command = command
            
            # 确保命令以换行符结尾
            if not command.endswith('\n'):
                command += '\n'
                
            self.ser.write(command.encode('utf-8'))
            print(f"发送: {command.strip()}")
            return True
            
        except Exception as e:
            print(f"发送错误: {e}")
            return False
    
    def set_target(self, position):
        """设置目标位置"""
        if 0 <= position <= 1023:
            return self.send_command(str(position))
        else:
            print("目标位置必须在 0-1023 范围内")
            return False
    
    def stop_system(self):
        """停止系统"""
        return self.send_command("stop")
    
    def start_system(self):
        """启动系统"""
        return self.send_command("start")
    
    def get_status(self):
        """获取状态"""
        return self.send_command("status")
    
    def debug_info(self):
        """获取调试信息"""
        return self.send_command("debug")
    
    def motor_test(self):
        """电机测试"""
        return self.send_command("test")
    
    def toggle_silent_mode(self):
        """切换静默模式"""
        self.silent_mode = not self.silent_mode
        status = "开启" if self.silent_mode else "关闭"
        print(f"静默模式已{status}")
        return self.silent_mode

def list_serial_ports():
    """列出可用的串口"""
    ports = []
    if sys.platform.startswith('win'):
        # Windows
        for i in range(1, 257):
            try:
                port = f"COM{i}"
                ser = serial.Serial(port)
                ser.close()
                ports.append(port)
            except (OSError, serial.SerialException):
                pass
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # Linux
        ports = [f"/dev/{device}" for device in os.listdir('/dev') 
                if device.startswith('ttyUSB') or device.startswith('ttyACM')]
    elif sys.platform.startswith('darwin'):
        # macOS
        ports = [f"/dev/{device}" for device in os.listdir('/dev') 
                if device.startswith('tty.usb') or device.startswith('ttyACM')]
    return ports

def clear_screen():
    """清屏"""
    os.system('cls' if os.name == 'nt' else 'clear')

def print_menu(controller):
    """打印菜单"""
    silent_status = "开启" if controller.silent_mode else "关闭"
    
    print("\n" + "="*50)
    print("          电机控制系统 - Python控制端")
    print("="*50)
    print(f"静默模式: {silent_status} (状态信息将{'不' if controller.silent_mode else ''}显示)")
    print("-"*50)
    print("1.  设置目标位置")
    print("2.  启动系统")
    print("3.  停止系统")
    print("4.  获取状态")
    print("5.  调试信息")
    print("6.  电机测试")
    print("7.  手动输入命令")
    print("8.  切换静默模式")
    print("9.  重新连接")
    print("0.  退出")
    print("="*50)

def main():
    # 自动检测串口
    ports = list_serial_ports()
    if not ports:
        print("未找到可用的串口设备")
        return
    
    print("检测到的串口:")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port}")
    
    # 选择串口
    while True:
        try:
            choice = input(f"选择串口 (1-{len(ports)}): ")
            port_index = int(choice) - 1
            if 0 <= port_index < len(ports):
                selected_port = ports[port_index]
                break
            else:
                print("无效选择")
        except ValueError:
            print("请输入数字")
    
    # 创建控制器实例
    controller = MotorController(port=selected_port)
    
    # 连接
    if not controller.connect():
        print("连接失败，程序退出")
        return
    
    print("连接成功！输入命令控制电机系统")
    time.sleep(1)  # 等待初始信息接收完成
    
    try:
        while True:
            clear_screen()
            print_menu(controller)
            
            choice = input("请选择操作 (0-9): ").strip()
            
            if choice == '1':
                # 设置目标位置
                try:
                    position = int(input("输入目标位置 (0-1023): "))
                    controller.set_target(position)
                except ValueError:
                    print("请输入有效的数字")
                input("\n按回车键继续...")
                
            elif choice == '2':
                # 启动系统
                controller.start_system()
                input("\n按回车键继续...")
                
            elif choice == '3':
                # 停止系统
                controller.stop_system()
                input("\n按回车键继续...")
                
            elif choice == '4':
                # 获取状态
                controller.get_status()
                input("\n按回车键继续...")
                
            elif choice == '5':
                # 调试信息
                controller.debug_info()
                input("\n按回车键继续...")
                
            elif choice == '6':
                # 电机测试
                controller.motor_test()
                input("\n按回车键继续...")
                
            elif choice == '7':
                # 手动输入命令
                command = input("输入命令: ").strip()
                if command:
                    controller.send_command(command)
                input("\n按回车键继续...")
                
            elif choice == '8':
                # 切换静默模式
                controller.toggle_silent_mode()
                input("\n按回车键继续...")
                
            elif choice == '9':
                # 重新连接
                controller.disconnect()
                time.sleep(1)
                if not controller.connect():
                    print("重新连接失败")
                    break
                input("\n重新连接成功，按回车键继续...")
                
            elif choice == '0':
                # 退出
                print("退出程序...")
                break
                
            else:
                print("无效选择")
                input("\n按回车键继续...")
                
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        controller.disconnect()

# 简化版本 - 直接命令行控制
def simple_control():
    """简化版本，直接命令行控制"""
    ports = list_serial_ports()
    if not ports:
        print("未找到可用的串口设备")
        return
    
    print("检测到的串口:")
    for i, port in enumerate(ports):
        print(f"{i+1}. {port}")
    
    # 选择串口
    while True:
        try:
            choice = input(f"选择串口 (1-{len(ports)}): ")
            port_index = int(choice) - 1
            if 0 <= port_index < len(ports):
                selected_port = ports[port_index]
                break
            else:
                print("无效选择")
        except ValueError:
            print("请输入数字")
    
    controller = MotorController(port=selected_port)
    
    if not controller.connect():
        print("连接失败，程序退出")
        return
    
    print("连接成功！输入命令控制电机系统")
    print("命令格式: [目标位置] 或 stop/start/status/debug/test/silent")
    print("输入 'quit' 退出程序")
    
    try:
        while True:
            command = input("> ").strip()
            
            if command == 'quit':
                break
            elif command == 'silent':
                controller.toggle_silent_mode()
            elif command.isdigit():
                position = int(command)
                if 0 <= position <= 1023:
                    controller.set_target(position)
                else:
                    print("目标位置必须在 0-1023 范围内")
            elif command in ['stop', 'start', 'status', 'debug', 'test']:
                controller.send_command(command)
            else:
                print("未知命令")
                
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        controller.disconnect()

if __name__ == "__main__":
    # 询问用户使用哪种界面
    print("选择控制界面:")
    print("1. 图形菜单界面")
    print("2. 简单命令行界面")
    
    choice = input("请选择 (1/2): ").strip()
    
    if choice == '2':
        simple_control()
    else:
        main()