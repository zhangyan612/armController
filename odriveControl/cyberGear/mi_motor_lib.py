import can
from CyberGearDriver import CyberGearMotor, RunMode, CyberMotorMessage
import time

# Connect to the bus with python-can
# @see https://python-can.readthedocs.io/en/stable/installation.html
bus = can.interface.Bus(
    interface='socketcan',
    channel='can0',
    bitrate=1000000,
)

# Create function to pass the messages from CyberGear to the CAN bus
def send_message(message: CyberMotorMessage):
    bus.send(
        can.Message(
            arbitration_id=message.arbitration_id,
            data=message.data,
            is_extended_id=message.is_extended_id,
        )
    )

# Create the motor controller
motor = CyberGearMotor(motor_id=6, send_message=send_message)

# def has_fault():
#     active_faults = [motor.fault for fault, is_active in faults.items() if is_active]
#     print("Active motor faults:", motor.faults)

# def faults_clear():
#     print("No more faults")

# # Define event listeners
# motor.on("has_fault", has_fault)
# motor.on("fault_cleared", has_fault)

# Check regularly
# motor.request_motor_fault_status()

# Send the CyberGear driver messages received from the CAN bus
# notifier = can.Notifier(bus, [motor.message_received])

# print(notifier)
# faults = motor.request_motor_fault_status()

# active_faults = [motor.fault for fault, is_active in faults.items() if is_active]
# print("Active motor faults:", motor.faults)

# motor.enable()

# motor.mode(RunMode.OPERATION_CONTROL)
# motor.enable()

# # Move the motor back and forth
# while True:
#     print("Move to position -6")
#     motor.control(position=-6, velocity=0, torque=0, kp=0.1, kd=0.1)
#     time.sleep(2)

#     print("Move to position 6")
#     motor.control(position=6, velocity=0, torque=0, kp=0.1, kd=0.1)
#     time.sleep(2)



# # torque_mode
# # Init motor
# motor.mode(RunMode.TORQUE)
# motor.enable()

# # Apply 0.5A of torque to the motor
# print("0.5A")
# motor.set_parameter("iq_ref", 0.5)

# while True:
#     time.sleep(5)


# #velocity_mode
# # Init motor
# motor.mode(RunMode.VELOCITY)
# motor.enable()

# # Alternate the motor speed
# while True:
#     print("5 rad/s")
#     motor.set_parameter("spd_ref", 5)
#     time.sleep(2)

#     print("10 rad/s")
#     motor.set_parameter("spd_ref", 10)
#     time.sleep(2)


#positon mode

# # Init motor
# motor.mode(RunMode.POSITION)
# motor.enable()
# motor.set_parameter("limit_spd", 10)

# # Move the motor back and forth
# while True:
#     print("Move to position -5")
#     motor.set_parameter("loc_ref", -5)
#     time.sleep(2)

#     print("Move to position 5")
#     motor.set_parameter("loc_ref", 5)
#     time.sleep(2)

motor.set_zero_position()


# Get the motor state
motor.request_motor_state()
time.sleep(1)
print(motor.state.get("position"))

motor.request_parameter("spd_ref")
time.sleep(1)
print(motor.params.get("spd_ref"))

# motor.enable()

# def state_updated():
#     print(f"{motor.state}")

# motor.on("state_changed", state_updated)
# motor.request_motor_state()