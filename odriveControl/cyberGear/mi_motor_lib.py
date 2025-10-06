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

motor.enable()
motor.mode(RunMode.POSITION)

# Limit the top speed
# motor.set_parameter("limit_spd", 10)

# # Move to position 8 rad
# motor.set_parameter("loc_ref", 8)
# 

motor.request_motor_state()
time.sleep(1)
print(motor.state.get("position"))


motor.request_parameter("loc_ref")
time.sleep(1)
print(motor.params.get("loc_ref"))