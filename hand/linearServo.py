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




def construct_message(id_value, pwm_value, time_value):
    if not (isinstance(pwm_value, int) and isinstance(time_value, int)):
        raise ValueError("PWM and Time values must be integers.")
    pwm_str = f"{pwm_value:04}"  # Format PWM as a 4-digit string
    time_str = f"{time_value:04}"  # Format Time as a 4-digit string
    return f"#{id_value}P{pwm_str}T{time_str}!".encode('utf-8')


def Grow():
    grow = b'#001P0500T1000!'
    ser.write(grow)
    # for i in range(4):
    #     print(f"Iteration {i+1}")
    #     ser.write(grow)
    #     time.sleep(0.9)

def Shrink():
    shrink = b'#001P2500T1000!'
    for i in range(4):
        print(f"Iteration {i+1}")
        ser.write(shrink)
        time.sleep(0.9)

# pwm 500 small, pwm 2500 big
def Move_Motor(id, pwm, time):
    msg = construct_message(id, pwm, time)
    print(msg)
    ser.write(msg)

# # ser.flushInput()  # 清空接收缓存
# # portRead()  # 将单线串口配置为输入
# time.sleep(0.005)  # 稍作延时，等待接收完毕 重要
# count = ser.inWaiting()    # 获取接收缓存中的字节数
# if count != 0:  # 如果接收到的数据不空
#     recv_data = ser.read(count)  # 读取接收到的数据
#     print(recv_data)


if __name__ == "__main__":
    # Grow()
    # Shrink()
    id_value = '001'
    pwm_value = 500
    time_value = 5000
    Move_Motor(id_value, pwm_value, time_value)

    # id_value = '002'
    # pwm_value = 1600
    # time_value = 20
    # Move_Motor(id_value, pwm_value, time_value)

    # time.sleep(0.3)
    # id_value = '002'
    # pwm_value = 500
    # time_value = 1000
    # Move_Motor(id_value, pwm_value, time_value)

