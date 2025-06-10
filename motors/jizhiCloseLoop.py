import jizhiMotorControl
import time
import sys
import os

parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(parent_dir)
from encoder import rs485EncoderReader

def startVelocityMotor(mc):
    mc.send_precise_control(p=1, v=0, kp=0.0, kd=0.0, t=0.05)
    mc.monitor(timeout=0.05)
    mc.send_command('velocity_mode')
    mc.monitor(timeout=0.05)
    mc.send_command('start_motor')
    mc.monitor(timeout=0.05)

def stopMotor(mc):
    mc.send_command('stop_motor')
    mc.monitor(timeout=0.05)

def moveToTargetPosition(mc, encoder, target_position, max_velocity=5.0, threshold=10, resolution=16384):
    startVelocityMotor(mc)
    print("Moving to target position:", target_position)

    while True:
        angle, status = encoder.get_position()
        if angle is None:
            print("Encoder read failed, skipping.")
            continue

        # Calculate shortest-path error
        error = (target_position - angle + resolution // 2) % resolution - resolution // 2
        print(f"Current: {angle}, Error: {error}")

        if abs(error) <= threshold:
            print("Target reached.")
            break

        # 1. FIXED SIGN: Negative gain for correct direction
        # 2. Adaptive gain: Reduce near target to prevent overshoot
        adaptive_gain = 0.02 * min(1.0, max(0.1, abs(error)/500.0))
        velocity_command = -adaptive_gain * error  # CORRECTED SIGN

        # 3. Smooth deceleration near target
        velocity_command = max(-max_velocity, min(max_velocity, velocity_command))
        if abs(error) < 500:
            velocity_command *= 0.7  # Reduce speed near target

        # 4. Match command duration to control period (50ms)
        mc.send_precise_control(p=1, v=velocity_command, kp=0.0, kd=0.0, t=0.05)
        time.sleep(0.05)

    stopMotor(mc)

if __name__ == "__main__":
    mc = jizhiMotorControl.MotorController()
    encoder = rs485EncoderReader.EncoderReader(port='COM5')
    try:
        angle, _ = encoder.get_position()
        print(f"Initial Encoder value: {angle}")
        if angle is not None:
            target_position = 2000  
            moveToTargetPosition(mc, encoder, target_position, 
                                max_velocity=5.0, 
                                threshold=100)
    finally:
        encoder.close()
        stopMotor(mc)