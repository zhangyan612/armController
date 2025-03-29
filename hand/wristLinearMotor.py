import time
import serial

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
    print("Serial port is open")
else:
    print("Failed to open serial port")


def construct_message(id_value, pwm_value, time_value):
    # 检查pwm_value和time_value是否为整数
    if not (isinstance(pwm_value, int) and isinstance(time_value, int)):
        raise ValueError("PWM and Time values must be integers.")
    pwm_str = f"{pwm_value:04}"  # Format PWM as a 4-digit string
    time_str = f"{time_value:05}"  # Format Time as a 4-digit string
    return f"#{id_value}#{pwm_str}#{time_str}#".encode('utf-8')


#02#1600#00010#

def Grow():
    grow = b'#01#1600#00010#'
    ser.write(grow)
    # for i in range(4):
    #     print(f"Iteration {i+1}")
    #     ser.write(grow)
    #     time.sleep(0.9)

def Shrink():
    shrink = b'#02#3000#00010#'
    ser.write(shrink)
    # for i in range(4):
    #     print(f"Iteration {i+1}")
    #     ser.write(shrink)
    #     time.sleep(0.9)

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
def shoulderRight(turn, time_value):
    if turn == "left":
        id_value = '06'
        pwm_value = 5000  # 松
        Move_Motor(id_value, pwm_value, time_value)
        time.sleep(0.01)
        id_value = '01'
        pwm_value = 5000  # 拉
        Move_Motor(id_value, pwm_value, time_value)
    if turn == "right":
        id_value = '01'
        pwm_value = 0  # 松
        Move_Motor(id_value, pwm_value, time_value)
        time.sleep(0.01)
        id_value = '06'
        pwm_value = 0 # 拉
        Move_Motor(id_value, pwm_value, time_value)


def lowerRight(turn, time_value):
    if turn == "left":
        id_value = '04'
        pwm_value = 0 # 松
        Move_Motor(id_value, pwm_value, time_value)
        time.sleep(0.01)
        id_value = '05'
        pwm_value = 5000 # 拉
        Move_Motor(id_value, pwm_value, time_value)
    if turn == "right":
        id_value = '05'
        pwm_value = 0  # 松
        Move_Motor(id_value, pwm_value, time_value)
        time.sleep(0.01)
        id_value = '04'
        pwm_value = 5000 # 拉
        Move_Motor(id_value, pwm_value, time_value)


if __name__ == "__main__":
    # Grow()
    # time.sleep(0.1)
    # Shrink()

    # 1000 grow 2100 shrink
    # 2000 is the divider
    #small linear motor 1- 5000

    timeChange = 2

    # shoulderRight("right", timeChange)
    # shoulderRight("left", timeChange)
    # lowerRight("right", timeChange)
    lowerRight("left", timeChange)
    

    # id_value = '06'
    # pwm_value = 0 # 拉
    # # pwm_value = 5000  # 松

    # time_value = timeChange
    # Move_Motor(id_value, pwm_value, time_value)

    # time.sleep(0.01)

    # id_value = '01'
    # pwm_value = 5000  # 拉
    # # pwm_value = 1000  # 松

    # time_value = timeChange
    # Move_Motor(id_value, pwm_value, time_value)
    # time.sleep(0.01)


    # id_value = '02'
    # pwm_value = 1000  # 松
    # # pwm_value = 5000  # 拉
    # time_value = 5
    # Move_Motor(id_value, pwm_value, time_value)
    # time.sleep(0.01)

    # id_value = '03'
    # pwm_value = 0  # 松
    # # pwm_value = 5000  # 拉
    # time_value = 5
    # Move_Motor(id_value, pwm_value, time_value)
    # time.sleep(5)

    # id_value = '03'
    # pwm_value = 3000  # 拉
    # time_value = 5
    # Move_Motor(id_value, pwm_value, time_value)
    # time.sleep(0.01)


    # id_value = '04'
    # pwm_value = 1000 # 松
    # # pwm_value = 5000 # 拉
    # time_value = 3
    # Move_Motor(id_value, pwm_value, time_value)

    # time.sleep(0.01)

    # id_value = '05'
    # # pwm_value = 0 # 松
    # pwm_value = 5000 # 拉
    # time_value = 3
    # Move_Motor(id_value, pwm_value, time_value)

