import odrive
odrv0 = odrive.find_any()

# odrv0.config.enable_can_a = True
# odrv0.axis0.requested_state = AXIS_STATE_IDLE
# odrv0.save_configuration()

# odrv0.axis0.controller.config.control_mode
# 控制模式。
# 0：电压控制
# 1：力矩控制
# 2：速度控制
# 3：位置控制
odrive.utils.dump_errors(odrv0)

odrv0.clear_errors()


# 位置控制失败时候，出现max current error, 可以把电流调到50A

odrv0.config.dc_max_positive_current = 50

# 位置控制失败时候，出现low current error, 先进入速度环，根据encoder 目前位置调到接近0点

odrv0.axis0.controller.config.control_mode=CONTROL_MODE_VELOCITY_CONTROL
odrv0.axis0.controller.config.input_mode=INPUT_MODE_VEL_RAMP
odrv0.axis0.requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_vel=15

odrv0.axis0.controller.input_vel=0

# encoder 目前位置
odrv0.axis0.encoder.pos_estimate
odrv0.axis0.encoder.config.index_offset = odrv0.axis0.encoder.pos_estimate

odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
odrv0.axis0.controller.input_pos=0

# 当前力矩
odrv0.axis0.controller.torque_setpoint


# odrv1.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
# odrv1.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
# odrv1.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL
# odrv1.axis0.controller.input_pos=20


# odrv0.axis0.requested_state=1 停止电机，进⼊空闲状态
# odrv0.axis0.requested_state=8 启动电机，进⼊闭环状态

# odrv0.axis0.requested_state=7 
# 对编码器进行校准。执行此操作前，请确认电机输出轴没有任何负载，且用手或其他装置固定住电机。此操作执行后，电机会正向和反向旋转一定时间，对编码器进行识别和校准。
# 在电机停转后，运行dump_errors(odrv0)查看错误，确认没有错误再进行其他后续步骤。

# 上电校准
# odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
# dump_errors(odrv0)
# odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
# dump_errors(odrv0)
# odrv0.axis0.motor.config.pre_calibrated = 1
# odrv0.axis0.encoder.config.pre_calibrated = 1
# odrv0.save_configuration()

# node_id：代表这个电机在总线上的唯一 ID，可在 odrivetool 中用
# odrv0.axis0.config.can.node_id 来读取和设置。
# nodeid = 0

# odrv0.axis0.config.can.node_id   0

# Connected to ODrive v3.7 86640B6C3537 (firmware v0.5.7) as odrv0

# 默认通信接口是 CAN，最大通信速率 1Mbps
#  odrv0.can.config.baud_rate    500000


# odrv1.axis0.controller.config.control_mode=CONTROL_MODE_POSITION_CONTROL
# odrv1.axis0.controller.config.input_mode=INPUT_MODE_POS_FILTER
# odrv1.axis0.requested_state=AXIS_STATE_CLOSED_LOOP_CONTROL
# odrv1.axis0.controller.input_pos=0


