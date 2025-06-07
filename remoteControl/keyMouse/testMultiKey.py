import time
import json
from pynput import mouse, keyboard
import math
import ch9329Comm
import serial

# Setup serial and ch9329
serial.ser = serial.Serial('COM10', 9600)

mouse_dev = ch9329Comm.mouse.DataComm(1920, 1080)
keyboard_dev = ch9329Comm.keyboard.DataComm()


# 键盘输出helloworld

keyboard_dev.send_multiple_keys((""), modifiers=["win"])  # 按下HELLO



serial.ser.close()  # 关闭串口
