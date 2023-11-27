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

ser.write(b'\x02')  # '2' in hexadecimal, backward
time.sleep(1)

ser.write(b'\x00')  # '0' in hexadecimal, stop

print("data sent")

# ser.flushInput()  # 清空接收缓存
# portRead()  # 将单线串口配置为输入
time.sleep(0.005)  # 稍作延时，等待接收完毕 重要
count = ser.inWaiting()    # 获取接收缓存中的字节数
if count != 0:  # 如果接收到的数据不空
    recv_data = ser.read(count)  # 读取接收到的数据
    print(recv_data)



# 9740
# 10000
# 10260

# 10520

# 10260
# 9740
# 9480
# 9220
# 8960