import serial
import time
# ====== 配置参数 ======
PORT = "COM10"
BAUDRATE = 115200
TIMEOUT = 1
COMMAND_DELAY = 0.2  # 指令间隔时间（秒）

# ====== 指令定义 ======
COMMANDS = {
    "clamp_min": "01 FB 00 01 F4 00 00 2A 94 01 00 6B",  # 夹紧到最小 （十六进制 0：地址，1：功能码，2：方向，3-4：速度*10，5-8：位置*10，9：0相对运动1绝对运动，10：多机同步标志，11：固定效验码
    "clamp_max": "01 FB 01 01 F4 00 00 00 00 01 00 6B",  # 松开到最大
    "motor_enable": "01 F3 AB 01 00 6B",                 # 电机使能
    "release_block": "01 0E 52 6B",                      # 解除堵转
    "anger_clear": "01 0A 6D 6B",                        # 角度清零
    "current_read": "01 27 6B"                           # 相电流读取
}

# ====== 全局变量声明 ======
ser = None  # 初始化全局变量


def initialize_serial():
    """初始化串口连接"""
    global ser  # 在函数开头声明全局变量
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=TIMEOUT
        )
        print(f"[{time.strftime('%H:%M:%S')}] 成功连接到 {PORT}")
        if not ser.is_open:
            ser.open()
            print(f"[{time.strftime('%H:%M:%S')}] 串口已打开")
    except serial.SerialException as e:
        print(f"[{time.strftime('%H:%M:%S')}] 串口初始化失败: {str(e)}")


def send_command(command_name):
    """发送指定名称的指令"""
    global ser
    if ser is None or not ser.is_open:
        print("[{time.strftime('%H:%M:%S')}] 串口未连接，指令发送失败")
        return False

    command = COMMANDS.get(command_name)
    if not command:
        print(f"[{time.strftime('%H:%M:%S')}] 未找到指令: {command_name}")
        return False

    try:
        data = bytes.fromhex(command)
        ser.write(data)
        print(f"[{time.strftime('%H:%M:%S')}] 发送成功: {command_name}")
        time.sleep(COMMAND_DELAY)
        return True
    except Exception as e:
        print(f"[{time.strftime('%H:%M:%S')}] 发送失败: {str(e)}")
        time.sleep(1)  # 错误后等待1秒
        return False


def grab_action():
    send_command("clamp_min")
    time.sleep(5)


def release_action():
    send_command("clamp_max")
    time.sleep(5)


def recover_action():
    send_command("release_block")
    time.sleep(0.3)


def enable_action():
    send_command("motor_enable")
    time.sleep(0.3)