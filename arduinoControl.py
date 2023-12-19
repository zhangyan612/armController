import time
import serial

# Write data to the serial port
def send_command(command):
    ser.write((command + '\n').encode())  # Send the command to the Arduino

# Create a serial object
ser = serial.Serial(
    port='COM5',  # replace with your port
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

time.sleep(2)  # Add a delay to aviod data missing 

# Check if the serial port is open
if ser.isOpen():
    print("Serial port is open")
else:
    print("Failed to open serial port")


send_command('f')  # Send the 'forward' command

time.sleep(2)

send_command("b")  # Send the 'backward' command

time.sleep(2)

send_command("s")  # Send the 'forward' command

# ser.flushInput()  # 清空接收缓存
# portRead()  # 将单线串口配置为输入
time.sleep(0.005)  # 稍作延时，等待接收完毕 重要
count = ser.inWaiting()    # 获取接收缓存中的字节数
if count != 0:  # 如果接收到的数据不空
    recv_data = ser.read(count)  # 读取接收到的数据
    print(recv_data)

