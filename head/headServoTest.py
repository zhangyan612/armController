import time
import serial


# def getServoPort():
#         ports = serial.tools.list_ports.comports()
#         for port, desc, hwid in sorted(ports):
#                 if 'USB-SERIAL CH340' in desc:
#                         return port
#         print('Error: Servo is not connected to any port')
#         return None

# port = getServoPort()

# Create a serial object
ser = serial.Serial(
    port='COM10',  # replace with your port
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

# Check if the serial port is open
if ser.isOpen():
    print("Head Serial port is open")
else:
    print("Failed to open head serial port")

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

def moveHeadServo(id, angle, duration):
    moveCommand = f"#{id}P{angle:04}T{duration:04}!" #'#002P1500T0316!'
    ser.write(moveCommand.encode())

def readHeadPosition(id):
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

moveHeadServo('002', 1500, 100)

time.sleep(1)

position = readHeadPosition('002')
print(position)




# #000PRAD!  read position
# response #000P1846! 