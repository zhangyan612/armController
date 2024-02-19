import time
import serial
from serial.tools import list_ports
import platform 

class LiftController:
    def __init__(self) -> None:
        self.port = self.getServoPort()

        # print(self.port)
        # Create a serial object
        self.ser = serial.Serial(
            port=self.port,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )
        # Check if the serial port is open
        if self.ser.isOpen():
            print("Lift serial port is open")
        else:
            print("Failed to open Lift serial port")
        time.sleep(2)  # Add a delay to aviod data missing 

    def getServoPort(self):
        p = platform.platform().lower()
        if 'linux' in p:
            port = '/dev/ttyUSB0'
            return port
        else:
            ports = list_ports.comports()
            for port, desc, hwid in sorted(ports):
                    if 'USB-SERIAL CH340' in desc:
                            return port
            print('Error: Servo is not connected to any port')
            return None

    # Write data to the serial port
    def send_command(self, command):
        print('sending command:'+ command)
        self.ser.write((command + '\n').encode())  # Send the command to the Arduino

    def goUp(self, seconds):
        self.send_command('f')  # forward
        time.sleep(seconds)

    def goDown(self, seconds):
        self.send_command('b')  # backward
        time.sleep(seconds)

    def stop(self):
        self.send_command('s')  # Stop


if __name__ == '__main__':
    ctrl = LiftController()
    ctrl.goUp(5)
    ctrl.goDown(5)
    ctrl.stop()

    # config needed to get permission
    # sudo usermod -a -G tty wheeltec
    # sudo usermod -a -G dialout wheeltec

# ser.flushInput()  # 清空接收缓存
# portRead()  # 将单线串口配置为输入
# time.sleep(0.005)  # 稍作延时，等待接收完毕 重要
# count = ser.inWaiting()    # 获取接收缓存中的字节数
# if count != 0:  # 如果接收到的数据不空
#     recv_data = ser.read(count)  # 读取接收到的数据
#     print(recv_data)

