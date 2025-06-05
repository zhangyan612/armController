import time
import json
import serial
from linear_motor_controller import MotorController

encoderPort = 'COM10'
ser = serial.Serial(encoderPort, 115200, timeout=1)
motor = MotorController()

# motor_id = '01'
# encoder_index = 0  # Encoder index to monitor

def get_angle(encoder_index):
    try:
        line = ser.readline().decode().strip()
        angles = json.loads(line)
        return angles[encoder_index] if angles else None
    except json.JSONDecodeError:
        return None

def wait_for_valid_angle(encoder_index):
    while True:
        angle = get_angle(encoder_index)
        if angle is not None and angle != -1:
            return angle

def move_and_monitor(pwm, duration_ms):
    motor.move_motor("01", pwm, duration_ms)
    motor.move_motor("02", pwm, duration_ms)

    start_time = time.time()
    last_angle = None

    print("Angle readings during motion:")
    while (time.time() - start_time) < (duration_ms/10) + 1:
        angle = get_angle(0)
        if angle is not None and angle != -1:
            last_angle = angle
            print(f"{angle:.2f} deg")
        time.sleep(0.01)  # 10 ms sampling

    return last_angle


def determine_up_down_pwm(initial_angle, target_angle):
    """
    Returns the PWM value based on angle direction.
    If initial < target: move forward with pwm=0
    If initial >= target: move backward with pwm=3000
    """
    if initial_angle < target_angle:
        return 0
    else:
        return 3000

def calculate_duration(initial_angle, target_angle):
    """
    Calculates the motor movement duration based on angle difference.
    Uses 0.5 duration units per degree of movement.
    """
    degrees_to_move = abs(target_angle - initial_angle)
    duration = degrees_to_move * 0.5  # 0.5 ms per degree
    return max(int(duration), 1)  # ensure at least 1 ms duration

def move_same_direction_stop(idList, pwm, duration_ms, target_angle):
    motor.move_multi_motor(idList, pwm, duration_ms)

    start_time = time.time()
    last_angle = None

    print("Angle readings during motion:")
    while (time.time() - start_time) < (duration_ms / 10) + 1:
        angle = get_angle(0)
        if angle is not None and angle != -1:
            last_angle = angle
            print(f"{angle:.2f} deg")
            error = target_angle - last_angle
            if abs(error) < 3.0:
                motor.stop_motor(idList)
                print(f"Target angle {target_angle:.2f} reached with error {error:.2f}")
                break
        time.sleep(0.01)  # 10 ms sampling
    return last_angle


def move_opposite_direction_stop(id1, id2, pwm1, pwm2, duration_ms, target_angle):
    motor_commands = [
        (id1, pwm1, duration_ms),  
        (id2, pwm2, duration_ms),
    ]
    message = motor.construct_multi_message(motor_commands)
    print(f"Sending message: {message}")
    motor.sendMsg(message)

    start_time = time.time()
    last_angle = None

    print("Angle readings during motion:")
    while (time.time() - start_time) < (duration_ms / 10) + 1:
        angle = get_angle(4)
        if angle is not None and angle != -1:
            last_angle = angle
            print(f"{angle:.2f} deg")
            error = target_angle - last_angle
            if abs(error) < 3.0:
                # motor.stop_motor([id1, id2])
                print(f"Target angle {target_angle:.2f} reached with error {error:.2f}")
                break
        time.sleep(0.01)  # 10 ms sampling
    return last_angle


def closeLoopTestUpDown(target):
    # duration = 5  # ms
    # target = 100
    min_angle = 70
    max_angle = 110

    # Clamp target_angle within limits
    target = max(min(target, max_angle), min_angle)

    idList = ['11', '12']

    initial_angle = wait_for_valid_angle(0)
    print(f"Initial angle: {initial_angle:.2f} degrees")

    pwm = determine_up_down_pwm(initial_angle, target)
    duration = calculate_duration(initial_angle, target)
    print(f"Moving with PWM: {pwm}, Duration: {duration} ms, Target: {target:.2f} degrees")

    final_angle = move_same_direction_stop(idList, pwm, duration, target)

    print(f"Final angle: {final_angle:.2f} degrees")



def determine_left_right_pwm(initial_angle, target_angle):
    """
    Determines PWM1 and PWM2 based on direction of movement.
    Moving right: pwm1 = 3000, pwm2 = 0
    Moving left:  pwm1 = 0,    pwm2 = 3000
    """
    if initial_angle < target_angle:
        return 3000, 0
    else:
        return 0, 3000

def calculate_left_right_duration(initial_angle, target_angle):
    """
    Estimate motor duration (in seconds) for movement based on observed nonlinear behavior.
    Uses piecewise estimation based on position.
    """
    degrees_to_move = abs(target_angle - initial_angle)

    # Estimate speed in deg/s based on position
    if initial_angle < 290:
        speed = 1.63  # deg/s
    elif initial_angle < 298:
        speed = 1.86
    elif initial_angle < 313:
        speed = 3.01
    else:
        speed = 2.00

    duration = degrees_to_move / speed
    return max(int(duration), 1)  # Ensure at least 1 second

def closeLoopTestLeftRight(target):
    # angle min: 275 degrees
    # Final angle max : 325 degrees
    min_angle = 275
    max_angle = 325

    # Clamp target_angle within limits
    target = max(min(target, max_angle), min_angle)

    # target = 300
    id1 = '11'
    id2 = '12'

    initial_angle = wait_for_valid_angle(4)
    print(f"Initial angle: {initial_angle:.2f} degrees")

    pwm1, pwm2 = determine_left_right_pwm(initial_angle, target)
    duration = calculate_left_right_duration(initial_angle, target)

    print(f"Moving with PWM1: {pwm1}, PWM2: {pwm2}, Duration: {duration} seconds, Target: {target:.2f} degrees")

    final_angle = move_opposite_direction_stop(id1, id2, pwm1, pwm2, duration, target)

    print(f"Final angle: {final_angle:.2f} degrees")
    degreeChange = final_angle - initial_angle
    print(f"For {duration} seconds, pwm1={pwm1}, pwm2={pwm2}, Motor moved from {initial_angle:.2f} to {final_angle:.2f}, Degree change: {degreeChange:.2f} degrees")

try:
    # flat angle
    targetLeftRight = 310  
    targetUpDown = 110 
    closeLoopTestLeftRight(targetLeftRight)
    closeLoopTestUpDown(targetUpDown)
finally:
    motor.close()

# angle min: 275 degrees
# angle max : 325 degrees

# For 5 seconds, pwm1=3000, pwm2=0, Motor move from 280.2 to 288.37, Degree change: 8.17 degrees
# For 5 seconds, pwm1=3000, pwm2=0, Motor move from 288.72 to 298.04, Degree change: 9.32 degrees
# For 5 seconds, pwm1=3000, pwm2=0, Motor move from 298.04 to 313.07, Degree change: 15.03 degrees
# For 5 seconds, pwm1=3000, pwm2=0, Motor move from 313.24 to 323.26, Degree change: 10.02 degrees