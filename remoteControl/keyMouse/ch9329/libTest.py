from serial import Serial

from ch9329 import Keyboard, Mouse

class CH9329:
    ser: Serial = None

    def __init__(self, port: str, baudrate: int, timeout: int = 1, screenx: int = 1920, screeny: int = 1080):
        """
        CH9329控制的集合
        :param port: COM端口
        :param baudrate: 波特率
        :param timeout: 超时时间
        :param screenx: 屏幕X轴
        :param screeny: 屏幕Y轴
        """
        self.ser = Serial(port, baudrate, timeout=timeout)
        self.keyboard = Keyboard(self.ser)
        self.mouse = Mouse(self.ser, screenx, screeny)

    def close(self):
        """释放串口"""
        self.ser.close()


ch9329 = CH9329("COM10", 9600, timeout=1, screenx=1920, screeny=1080)

# 自定义发送按键, 最多支持6个按键,modifiers为控制键,可选
# ch9329.keyboard.send(("ctrl", "alt", "del", "", "", ""), modifiers=[])
# ch9329.keyboard.send(("alt", "del", "", "", "", ""), modifiers=["ctrl"])

# 按下单个按键, modifiers为控制键,可选,不释放则为长按
# ch9329.keyboard.press("a", modifiers=["shift"])

# # 释放所有按键
# ch9329.keyboard.release()

# # 按下后自动释放按键, min_interval和max_interval为释放延迟, 默认0.02秒到0.06秒
# ch9329.keyboard.press_and_release("a", modifiers="shift", min_interval=0.02, max_interval=0.06)

# # 和send一样,多了个校验,确保不超出6个
# ch9329.keyboard.trigger_keys(["ctrl", "alt", "del", "", "", ""], modifiers=[])
# ch9329.keyboard.trigger_keys(["alt", "del", "", "", "", ""], modifiers=["ctrl"])

# # 多字符输入,适合输入多个按键
# ch9329.keyboard.write("Hello World\n")
# ch9329.keyboard.write("abcdefghijklmnopqrstuvwxyz\n")
# ch9329.keyboard.write("ABCDEFGHIJKLMNOPQRSTUVWXYZ\n")
# ch9329.keyboard.write("0123456789\n")
# ch9329.keyboard.write("!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~\n")

# # press的简单使用, 按下某个按键,长按不释放
# ch9329.keyboard.longpress("e")

# 绝对移动
ch9329.mouse.absolute_move(1458, 544)
