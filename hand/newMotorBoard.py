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
    time_str = f"{time_value:05}"  # Format Time as a 5-digit string
    return f"#{id_value}#{pwm_str}#{time_str}#".encode('utf-8')


def Grow():
    grow = b'#1#1000#00020#'
    ser.write(grow)

def Shrink():
    shrink = b'#1#3000#00020#'
    for i in range(4):
        print(f"Iteration {i+1}")
        ser.write(shrink)
        time.sleep(0.9)

# pwm 500 small, pwm 2500 big
def Move_Motor(id, pwm, time):
    msg = construct_message(id, pwm, time)
    print(msg)
    ser.write(msg)



if __name__ == "__main__":
    # Grow()
    # Shrink()
    id_value = '1'
    pwm_value = 3000
    time_value = 20
    Move_Motor(id_value, pwm_value, time_value)

