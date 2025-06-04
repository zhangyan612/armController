import time
import json
import serial
from linear_motor_controller import MotorController

encoderPort = 'COM9'
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



def move_stop(pwm, duration_ms, target_angle):
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
        error = target_angle - last_angle
        if abs(error) < 1.0:
            motor.move_motor("01", 2500, 1)
            motor.move_motor("02", 2500, 1)
    return last_angle

def pwm_test_sweep():
    pwm = 0
    duration = 10  # ms
    target = 110
    initial_angle = wait_for_valid_angle()
    print(f"Initial angle: {initial_angle:.2f} degrees")

    # final_angle = move_and_monitor(pwm, duration)
    # final_angle = move_stop(pwm, duration, target)

    # print(f"Final angle: {final_angle:.2f} degrees")

    # delta = final_angle - initial_angle
    # print(f"PWM: {pwm}, Degrees Moved: {delta:.2f}")


try:
    pwm_test_sweep()
finally:
    motor.close()
