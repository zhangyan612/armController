import serial
import time



PORT = "/dev/ttyUSB0"    
BAUDRATE = 115200
TIMEOUT = 1
COMMAND_DELAY = 0.2  

COMMANDS = {
    "clamp_min": "01 FB 00 01 20 00 00 2E 00 01 00 6B",
    "clamp_max": "01 FB 01 01 20 00 00 00 00 01 00 6B",
    "motor_enable": "01 F3 AB 01 00 6B",
    "release_block": "01 0E 52 6B",
    "anger_clear": "01 0A 6D 6B",
    "current_read": "01 27 6B"
}

ser = None


def initialize_serial():
    """初始化串口连接"""
    global ser  
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
        time.sleep(1) 
        return False


def grab_action():
    send_command("clamp_min")
    time.sleep(8)


def release_action():
    send_command("clamp_max")
    time.sleep(8)


def recover_action():
    send_command("release_block")
    time.sleep(0.3)


def enable_action():
    send_command("motor_enable")
    time.sleep(0.3)