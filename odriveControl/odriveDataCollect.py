import odrive
import numpy as np
odrv0 = odrive.find_any()
cap = odrive.utils.BulkCapture(lambda:[odrv0.axis0.motor.current_control.Iq_measured,odrv0.axis0.encoder.pos_estimate],data_rate=500,duration=2.5)
odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_pos=10

np.savetxt("test.csv",cap.data,delimiter=',')