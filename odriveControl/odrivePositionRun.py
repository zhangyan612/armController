import odrive
import time

odrv0 = odrive.find_any()

odrv0.axis0.controller.config.control_mode = odrive.utils.ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.input_mode = odrive.utils.InputMode.POS_FILTER
odrv0.axis0.requested_state = odrive.utils.AxisState.CLOSED_LOOP_CONTROL

for _ in range(10):  # Run 10 cycles
    odrv0.axis0.controller.input_pos = -12  # Move to -12
    time.sleep(5)  # Wait for a second (adjust as needed)
    odrv0.axis0.controller.input_pos = 11   # Move to 11
    time.sleep(5)  # Wait for a second (adjust as needed)
