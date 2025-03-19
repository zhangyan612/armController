import can
import time
import STW_MotorControl as odriveMotor

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

    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000) 
    print(f"Connected to can bus")

    motorID1 = 0x01      
    motorID2 = 0x02                       
    motorID3 = 0x03   
    motorID4 = 0x04     
    motorID5 = 0x05    

    # 执行校准 pass
    # calibrate_motor(bus,motorID1)             
    # calibrate_motor(bus,motorID1)             
    # calibrate_motor(bus,motorID3)      
    # calibrate_motor(bus,motorID4)
    # calibrate_motor(bus,motorID5)

    # all motor back to 0 position  pass
    # positionControlTest(bus, motorID1, 0)
    # positionControlTest(bus, motorID2, 0)
    # positionControlTest(bus, motorID3, 0)
    # positionControlTest(bus, motorID4, 0)
    positionControlTest(bus, motorID5, 0)

    # odriveMotor.clear_errors(bus, motorID5)

    # positionControlTest(bus, motorID5, 0)

    # speedLoopTest(bus, motorID1) pass
    # speedLoopTest(bus, motorID2) pass
    # speedLoopTest(bus, motorID3) pass
    # speedLoopTest(bus, motorID4) pass
    # speedLoopTest(bus, motorID5)

    # position = odriveMotor.get_encoder_estimates(bus, motorID4)
    # print(position)

    # err = odriveMotor.get_error(bus, motorID5, 0)
    # print(err)

    # odriveMotor.reboot_motor(bus, motorID4)

    # 在退出时确保资源被释放  
    bus.shutdown()


if __name__ == "__main__":
    runMotorTest()