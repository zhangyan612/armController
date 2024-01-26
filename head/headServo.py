import time
import serial

# Create a serial object
ser = serial.Serial(
    port='COM6',  # replace with your port
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

# Write data to the serial port


# ser.write(b'\x01')  # '1' in hexadecimal, forward
# time.sleep(1)

# ser.write(b'#001PRAD!')  # '2' in hexadecimal, backward
# time.sleep(1)

# # ser.write(b'\x00')  # '0' in hexadecimal, stop

# # print("data sent")

# # ser.flushInput()  # 清空接收缓存
# # portRead()  # 将单线串口配置为输入
# time.sleep(0.005)  # 稍作延时，等待接收完毕 重要
# count = ser.inWaiting()    # 获取接收缓存中的字节数
# if count != 0:  # 如果接收到的数据不空
#     recv_data = ser.read(count)  # 读取接收到的数据
#     print(recv_data)


def moveServo(id, angle, duration):
    moveCommand = f"#{id}P{angle:04}T{duration:04}!" #'#002P1500T0316!'
    ser.write(moveCommand.encode())

def readPosition(id):
    command = f'#{id}PRAD!'
    ser.write(command.encode())
    time.sleep(0.005)  # 稍作延时，等待接收完毕 重要
    count = ser.inWaiting()
    if count != 0:  # 如果接收到的数据不空
        recv_data = ser.read(count)  # 读取接收到的数据
        position = recv_data[5:9]
        return position

def resetServo(id):
    moveCommand = f"#{id}PSCK!" #'#002P1500T0316!'
    ser.write(moveCommand.encode())

moveServo('002', 1500, 100)

time.sleep(1)

position = readPosition('002')
print(position)