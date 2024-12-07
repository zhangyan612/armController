import odrive
import time

odrv0 = odrive.find_any()

odrive.utils.dump_errors(odrv0)
odrv0.clear_errors()
odrv0.axis0.requested_state=odrive.utils.AxisState.MOTOR_CALIBRATION
time.sleep(10)
while (odrv0.axis0.current_state!=1):
    time.sleep(0.5)
    odrive.utils.dump_errors(odrv0)
    odrv0.axis0.requested_state=odrive.utils.AxisState.ENCODER_OFFSET_CALIBRATION
    time.sleep(6)
    while (odrv0.axis0.current_state!=1):
        time.sleep(0.5)

odrive.utils.dump_errors(odrv0)
odrv0.axis0.motor.config.pre_calibrated=1
odrv0.axis0.encoder.config.pre_calibrated=1
odrv0.save_configuration()