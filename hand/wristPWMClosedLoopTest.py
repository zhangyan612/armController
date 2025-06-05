import time
import json
import serial
from linear_motor_controller import MotorController

encoderPort = 'COM10'
ser = serial.Serial(encoderPort, 115200, timeout=1)
motor = MotorController()

motor_id = '01'
encoder_index = 0  # Encoder index to monitor

def get_angle():
    try:
        line = ser.readline().decode().strip()
        angles = json.loads(line)
        return angles[encoder_index] if angles else None
    except json.JSONDecodeError:
        return None

def wait_for_valid_angle():
    while True:
        angle = get_angle()
        if angle is not None and angle != -1:
            return angle

def move_and_monitor(pwm, duration_ms):
    motor.move_motor("01", pwm, duration_ms)
    motor.move_motor("02", pwm, duration_ms)

    start_time = time.time()
    last_angle = None

    print("Angle readings during motion:")
    while (time.time() - start_time) < (duration_ms/10) + 1:
        angle = get_angle()
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

def move_stop(idList, pwm, duration_ms, target_angle):
    motor.move_multi_motor(idList, pwm, duration_ms)

    start_time = time.time()
    last_angle = None

    print("Angle readings during motion:")
    while (time.time() - start_time) < (duration_ms / 10) + 1:
        angle = get_angle()
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

def closeLoopTestUpDown():
    # duration = 5  # ms

    target = 90
    idList = ['11', '12']

    initial_angle = wait_for_valid_angle()
    print(f"Initial angle: {initial_angle:.2f} degrees")

    pwm = determine_up_down_pwm(initial_angle, target)
    duration = calculate_duration(initial_angle, target)
    print(f"Moving with PWM: {pwm}, Duration: {duration} ms, Target: {target:.2f} degrees")

    final_angle = move_stop(idList, pwm, duration, target)

    print(f"Final angle: {final_angle:.2f} degrees")

    # delta = final_angle - initial_angle
    # print(f"PWM: {pwm}, Degrees Moved: {delta:.2f}")


try:
    closeLoopTestUpDown()
finally:
    motor.close()
