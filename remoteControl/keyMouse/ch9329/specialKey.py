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


ch9329 = CH9329("COM12", 9600, timeout=1, screenx=1920, screeny=1080)

# 自定义发送按键, 最多支持6个按键,modifiers为控制键,可选
ch9329.keyboard.send(("ctrl", "alt", "del", "", "", ""), modifiers=[])
# ch9329.keyboard.send(("alt", "del", "", "", "", ""), modifiers=["ctrl"])


ch9329.keyboard.release()

# DD