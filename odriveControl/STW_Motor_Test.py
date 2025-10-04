import can
import time
import STW_MotorControl as odriveMotor
import platform  

def positionControlTest(bus, motorID, position):
    # # position control test success
    odriveMotor.set_controller_mode(bus, motorID, control_mode=3, input_mode=3)
    time.sleep(0.5)
    # Set to closed loop state
    odriveMotor.set_closed_loop_state(bus, motorID)
    time.sleep(0.5)
    # Set position
    odriveMotor.set_position(bus, motorID, position)


def speedControlTest(bus, motor_id, speed):
    odriveMotor.set_controller_mode(bus, motor_id, control_mode=2, input_mode=2)
    time.sleep(0.5)
    
    # Set to closed loop state
    odriveMotor.set_closed_loop_state(bus, motor_id)
    time.sleep(0.5)
    
    # Set velocity
    odriveMotor.set_velocity(bus, motor_id, speed, 0)
            

def speedLoopTest(bus, motor_id):
    speedControlTest(bus, motor_id, 10)
    time.sleep(5)
    speedControlTest(bus, motor_id, -10)
    time.sleep(5)
    speedControlTest(bus, motor_id, 0)



def runMotorTest():
    # remember to manually open can port before running code
    # sudo ip link set up can0
    # sudo ip link set can0 type can bitrate 500000 loopback off
    # sudo ip link set up can0
    if platform.system() == 'Windows':
        interface = 'pcan'
        channel = 'PCAN_USBBUS1'
    else:
        interface = 'socketcan'
        channel = 'can0'

    bus = can.interface.Bus(
        interface=interface,
        channel=channel,
        bitrate=500000
    )
    print(f"Connected to can bus")

    # bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000) 
    # print(f"Connected to can bus")

    motorID1 = 0x01     #shoulder1 
    motorID2 = 0x02     #shoulder2
    motorID3 = 0x03     #wrist 
    motorID4 = 0x04     #leg1
    motorID5 = 0x05     #leg2 
    motorID6 = 0x06     #leg3 
    motorID7 = 0x07     #leg4 
    motorID8 = 0x08     #ElbowRight  not stable 
    motorID9 = 0x09     #ElbowLeft

    motorID10 = 10     #leg5
    motorID11 = 11     #leg6 

    
    activeMotor = motorID10

    # 执行校准 pass
    # calibrate_motor(bus,motorID1)             
    # calibrate_motor(bus,motorID1)             
    # calibrate_motor(bus,motorID3)      
    # calibrate_motor(bus,motorID4)
    # calibrate_motor(bus,motorID5)
    position = odriveMotor.get_encoder_estimates(bus, activeMotor)
    print(position)
    err = odriveMotor.get_error(bus, activeMotor, 0)
    print(err)
    odriveMotor.clear_errors(bus, activeMotor)

    # odriveMotor.disable_can(bus, activeMotor)


    # # all motor back to 0 position  pass
    positionControlTest(bus, activeMotor, 0)
    # positionControlTest(bus, motorID2, 2)
    # positionControlTest(bus, motorID3, 2)
    # positionControlTest(bus, motorID4, 0)
    # positionControlTest(bus, motorID5, 0)
    # positionControlTest(bus, motorID6, 0)
    # positionControlTest(bus, motorID7, 0)

    time.sleep(3)
    positionControlTest(bus, activeMotor, 10)
    
    time.sleep(3)
    positionControlTest(bus, activeMotor, 0)
    time.sleep(3)

    # positionControlTest(bus, activeMotor, 0)

    # speedLoopTest(bus, motorID1) pass
    # speedLoopTest(bus, motorID2) pass
    # speedLoopTest(bus, motorID3) pass
    # speedLoopTest(bus, motorID4) pass
    # speedLoopTest(bus, motorID5)

    # position = odriveMotor.get_encoder_estimates(bus, activeMotor)
    # print(position)

    odriveMotor.set_axis_state(bus, activeMotor, 1) # IDLE

    # err = odriveMotor.get_error(bus, activeMotor, 0)
    # print(err)

    # odriveMotor.reboot_motor(bus, activeMotor)

    # 在退出时确保资源被释放  
    bus.shutdown()


if __name__ == "__main__":
    runMotorTest()