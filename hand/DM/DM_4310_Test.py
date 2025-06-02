import math
from DM_CAN import *
import serial
import time


Motor1=Motor(DM_Motor_Type.DM4310,0x01,0x11)
serial_device = serial.Serial('COM13', 921600, timeout=0.5)
MotorControl1=MotorControl(serial_device)
MotorControl1.addMotor(Motor1)

# if MotorControl1.switchControlMode(Motor1,Control_Type.POS_VEL):
#     print("switch POS_VEL success")

print("sub_ver:",MotorControl1.read_motor_param(Motor1,DM_variable.sub_ver))
print("Gr:",MotorControl1.read_motor_param(Motor1,DM_variable.Gr))

# if MotorControl1.change_motor_param(Motor1,DM_variable.KP_APR,54):
#     print("write success")
print("PMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.PMAX))
print("MST_ID:",MotorControl1.read_motor_param(Motor1,DM_variable.MST_ID))
print("VMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.VMAX))
print("TMAX:",MotorControl1.read_motor_param(Motor1,DM_variable.TMAX))

# MotorControl1.enable(Motor3)
# MotorControl1.save_motor_param(Motor1)

MotorControl1.enable(Motor1)
MotorControl1.enable_old(Motor1, Control_Type.MIT)

MotorControl1.refresh_motor_status(Motor1)
print("Motor1:","POS:", Motor1.getPosition(), "VEL:", Motor1.getVelocity(), "TORQUE:", Motor1.getTorque())
MotorControl1.controlMIT(Motor1, 50, 0.3, 0, 0, 0)

# i=0
# while i<10000:
#     q=math.sin(time.time())
#     i=i+1
#     # MotorControl1.control_pos_force(Motor1, 10, 1000,100)
#     # MotorControl1.control_Vel(Motor1, q*5)
#     MotorControl1.control_Pos_Vel(Motor1,q*8,30)
#     # print("Motor1:","POS:",Motor1.getPosition(),"VEL:",Motor1.getVelocity(),"TORQUE:",Motor1.getTorque())
#     MotorControl1.controlMIT(Motor1, 35, 0.1, 8*q, 0, 0)

#     # MotorControl1.control_Vel(Motor2, 8*q)
#     # print("Motor2:","POS:",Motor2.getPosition(),"VEL:",Motor2.getVelocity(),"TORQUE:",Motor2.getTorque())
#     # print(Motor1.getTorque())
#     # print(Motor2.getTorque())
#     time.sleep(0.001)
    # MotorControl1.control(Motor3, 50, 0.3, q, 0, 0)

#语句结束关闭串口
serial_device.close()