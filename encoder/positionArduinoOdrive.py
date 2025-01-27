import serial
import time

# Initialize serial communication (make sure to use the correct port)
ser = serial.Serial('COM15', 9600, timeout=1)

def map_value(input_value, in_min, in_max, out_min, out_max):
    # Check if input_value is outside the boundary
    if input_value < in_min or input_value > in_max:
        return None
    # Linear interpolation formula
    return (input_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def read_analog():
    try:
        # Read a line of data from the serial port
        sensor_value = ser.readline().decode().strip()
        return int(sensor_value)
    except ValueError:
        # Handle cases where the data is not a number
        return None


import odrive
odrv0 = odrive.find_any()

odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL


while True:
    sensor_value = read_analog()
    if sensor_value is not None:
        # print(sensor_value)
        motor_position = map_value(sensor_value, 20, 120, -12, 11)
        # print(motor_position)
        if motor_position != None:
            odrv0.axis0.controller.input_pos=motor_position
        time.sleep(0.001)



