import time
import json
import serial
from linear_motor_controller import MotorController


encoderPort='COM10'
ser = serial.Serial(encoderPort, 115200, timeout=1)


def get_encoder_angle():
    line = ser.readline().decode().strip()
    try:
        angles = json.loads(line)
        return angles  # List of 8 angles
    except json.JSONDecodeError:
        return None

def closed_loop_control(target_angle, idList=['11', '12'], encoder_index=1, kp=3.5):
    motor = MotorController()
    # Issue with figuring out pwm and duration
    # pwm = int(2000 + kp * error)  # Basic proportional control

    pwm = 3000
    duration = 5
    motor.move_multi_motor(idList, pwm, duration)

    try:
        for _ in range(20):  # Run for a fixed number of iterations
            angles = get_encoder_angle()

            if not angles:
                continue

            current_angle = angles[encoder_index]

            if current_angle == -1:
                continue

            error = target_angle - current_angle

            if abs(error) < 1.0:
                print("Target reached")
                break

            print(f"Current angle: {current_angle:.2f}, Target angle: {target_angle:.2f}, error: {error:.2f}, PWM: {pwm}")

            time.sleep(0.05)

    finally:
        motor.close()


if __name__ == "__main__":
    target = 90  # degrees
    closed_loop_control(target_angle=target, idList=['11', '12'], encoder_index=0)