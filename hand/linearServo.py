import time
import serial

# Create a serial object
ser = serial.Serial(
    port='COM14',  # replace with your port
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

# Check if the serial port is open
if ser.isOpen():
    print("Serial port is open")
else:
    print("Failed to open serial port")


def Grow():
    grow = b'#001P0500T1000!'
    for i in range(4):
        print(f"Iteration {i+1}")
        ser.write(grow)
        time.sleep(0.9)

def Shrink():
    shrink = b'#001P2500T1000!'
    for i in range(4):
        print(f"Iteration {i+1}")
        ser.write(shrink)
        time.sleep(0.9)


# # ser.flushInput()  # 清空接收缓存
# # portRead()  # 将单线串口配置为输入
# time.sleep(0.005)  # 稍作延时，等待接收完毕 重要
# count = ser.inWaiting()    # 获取接收缓存中的字节数
# if count != 0:  # 如果接收到的数据不空
#     recv_data = ser.read(count)  # 读取接收到的数据
#     print(recv_data)


if __name__ == "__main__":
    Grow()
    Shrink()