#!/usr/bin/python3
# -*- coding: UTF-8 -*-

import can
import time
import struct
import argparse

# Function to send CAN frame and handle response
def send_can_frame(bus, arbitration_id, data, block_receive=1):
    """
    Send a CAN frame and optionally wait for response
    
    Args:
        bus: CAN bus object
        arbitration_id: 11-bit CAN ID
        data: data bytes to send (list of 8 bytes)
        block_receive: whether to wait for response (0=no, 1=yes)
        
    Returns:
        tuple: (status, response data)
            - status: 0 if successful, 1 if error
            - rx_data: received data if block_receive=1, otherwise empty list
    """
    state = 0
    rx_data = [0 for i in range(8)]
    
    # Create and send the message
    message = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False,
        is_remote_frame=False
    )
    
    bus.send(message)
    
    if block_receive == 1:
        time_s = time.time()
        while True:
            message_ou = bus.recv(0.1)
            if message_ou is not None:
                rx_data = message_ou.data
                break
            elif time.time() - time_s > 1:
                print(f"CAN-ID: {hex(arbitration_id)} no response received - error")
                state = 1
                break
    
    return state, rx_data

# Calibrate the motor
def calibrate_motor(bus, motor_id):
    """
    Perform motor calibration
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
    """
    print(f"Calibrating motor {motor_id}...")
    
    # Motor calibration
    cmd_id = 0x07
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Send calibration command (4 = motor calibration)
    data = [0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data)
    
    time.sleep(3.0)  # Wait for motor calibration to complete
    
    # Encoder calibration
    cmd_id = 0x07
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Send encoder calibration command (7 = encoder calibration)
    data = [0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data)
    
    time.sleep(3.0)  # Wait for encoder calibration to complete
    print(f"Calibration of motor {motor_id} completed")

# Set controller mode
def set_controller_mode(bus, motor_id, control_mode, input_mode):
    """
    Set the controller mode for the motor
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        control_mode: Control mode (1=torque, 2=velocity, 3=position)
        input_mode: Input mode (1=passthrough, 2=vel ramp, 3=pos filter, 5=trap traj)
    """
    cmd_id = 0x0B
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Set controller mode for motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    data = [
        control_mode & 0xFF, 
        (control_mode >> 8) & 0xFF, 
        (control_mode >> 16) & 0xFF, 
        (control_mode >> 24) & 0xFF, 
        input_mode & 0xFF, 
        (input_mode >> 8) & 0xFF, 
        (input_mode >> 16) & 0xFF, 
        (input_mode >> 24) & 0xFF
    ]
    
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data)

# Set closed loop state
def set_closed_loop_state(bus, motor_id):
    """
    Set the motor to closed loop control state
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
    """
    cmd_id = 0x07
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Setting motor {motor_id} to closed loop state')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Command 8 = closed loop control
    data = [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data)

# Set velocity
def set_velocity(bus, motor_id, velocity, torque_ff=0):
    """
    Set the motor velocity
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        velocity: Target velocity in rev/s (float)
        torque_ff: Torque feedforward in Nm (float), default 0
    """
    cmd_id = 0x0D
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Setting velocity for motor {motor_id} to {velocity} rev/s')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Convert velocity to IEEE 754 float (little endian)
    vel_bytes = struct.pack('<f', velocity)
    
    # Convert torque_ff to IEEE 754 float (little endian)
    torque_bytes = struct.pack('<f', torque_ff)
    
    # Combine into data array
    data = list(vel_bytes) + list(torque_bytes)
    
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data, block_receive=0)

# Set position
def set_position(bus, motor_id, position, vel_ff=0, torque_ff=0):
    """
    Set the motor position
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        position: Target position in revolutions (float)
        vel_ff: Velocity feedforward in 0.001 rev/s (int16), default 0
        torque_ff: Torque feedforward in 0.001 Nm (int16), default 0
    """
    cmd_id = 0x0C
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Setting position for motor {motor_id} to {position} revolutions')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Convert position to IEEE 754 float (little endian)
    pos_bytes = struct.pack('<f', position)
    
    # Convert velocty and torque feedforward to int16 (little endian)
    vel_ff = int(vel_ff)
    torque_ff = int(torque_ff)
    
    # Ensure they fit in int16
    vel_ff = max(-32768, min(32767, vel_ff))
    torque_ff = max(-32768, min(32767, torque_ff))
    
    vel_bytes = struct.pack('<h', vel_ff)
    torque_bytes = struct.pack('<h', torque_ff)
    
    # Combine into data array
    data = list(pos_bytes) + list(vel_bytes) + list(torque_bytes)
    
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data, block_receive=0)

# Get encoder estimates (position and velocity)
def get_encoder_estimates(bus, motor_id):
    """
    Get encoder position and velocity estimates
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        
    Returns:
        tuple: (position, velocity) estimates
    """
    cmd_id = 0x09
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Requesting encoder estimates for motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Empty data for request
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    state, rx_data = send_can_frame(bus, arbitration_id, data)
    
    if state == 0 and len(rx_data) >= 8:
        # Convert bytes to floats (little endian)
        position = struct.unpack('<f', bytes(rx_data[0:4]))[0]
        velocity = struct.unpack('<f', bytes(rx_data[4:8]))[0]
        print(f'  Position: {position} rev, Velocity: {velocity} rev/s')
        return position, velocity
    else:
        print('  Failed to get encoder estimates')
        return None, None


# Reboot the motor
def reboot_motor(bus, motor_id):
    """
    Reboot the motor
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
    """
    cmd_id = 0x016
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Rebooting motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Empty data for reboot command
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data, block_receive=0)

# Get error information from the motor
def get_error(bus, motor_id, error_type):
    """
    Get error information from the motor
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        error_type: Error type (0=motor, 1=encoder, 2=sensorless, 3=controller)
        
    Returns:
        int: Error code
    """
    cmd_id = 0x003
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Getting error for motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Set error type
    data = [error_type, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    print(f'  Sending data: {data}')
    state, rx_data = send_can_frame(bus, arbitration_id, data)
    
    if state == 0 and len(rx_data) >= 4:
        error_code = struct.unpack('<I', bytes(rx_data[0:4]))[0]
        print(f'  Error code: {error_code}')
        return error_code
    else:
        print('  Failed to get error information')
        return None

# Clear errors on the motor
def clear_errors(bus, motor_id):
    """
    Clear errors on the motor
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
    """
    cmd_id = 0x018
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Clearing errors for motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Empty data for clear errors command
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data, block_receive=0)

# Set linear count for the encoder
def set_linear_count(bus, motor_id, linear_count):
    """
    Set linear count for the encoder
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        linear_count: Linear count value (int32)
    """
    cmd_id = 0x019
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Setting linear count for motor {motor_id} to {linear_count}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Convert linear count to int32 (little endian)
    linear_count_bytes = struct.pack('<i', linear_count)
    
    # Combine into data array
    data = list(linear_count_bytes) + [0x00, 0x00, 0x00, 0x00]
    
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data, block_receive=0)

# Get torque information from the motor
def get_torques(bus, motor_id):
    """
    Get torque information from the motor
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        
    Returns:
        tuple: (torque_setpoint, torque) in Nm
    """
    cmd_id = 0x01C
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Getting torques for motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Empty data for request
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    state, rx_data = send_can_frame(bus, arbitration_id, data)
    
    if state == 0 and len(rx_data) >= 8:
        torque_setpoint = struct.unpack('<f', bytes(rx_data[0:4]))[0]
        torque = struct.unpack('<f', bytes(rx_data[4:8]))[0]
        print(f'  Torque setpoint: {torque_setpoint} Nm, Torque: {torque} Nm')
        return torque_setpoint, torque
    else:
        print('  Failed to get torque information')
        return None, None

# Get power information from the motor
def get_powers(bus, motor_id):
    """
    Get power information from the motor
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        
    Returns:
        tuple: (electrical_power, mechanical_power) in W
    """
    cmd_id = 0x01D
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Getting powers for motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Empty data for request
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    state, rx_data = send_can_frame(bus, arbitration_id, data)
    
    if state == 0 and len(rx_data) >= 8:
        electrical_power = struct.unpack('<f', bytes(rx_data[0:4]))[0]
        mechanical_power = struct.unpack('<f', bytes(rx_data[4:8]))[0]
        print(f'  Electrical power: {electrical_power} W, Mechanical power: {mechanical_power} W')
        return electrical_power, mechanical_power
    else:
        print('  Failed to get power information')
        return None, None

# Get encoder estimates (position and velocity)
def get_encoder_estimates(bus, motor_id):
    """
    Get encoder position and velocity estimates
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        
    Returns:
        tuple: (position, velocity) estimates
    """
    cmd_id = 0x009
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Requesting encoder estimates for motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Empty data for request
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    state, rx_data = send_can_frame(bus, arbitration_id, data)
    
    if state == 0 and len(rx_data) >= 8:
        position = struct.unpack('<f', bytes(rx_data[0:4]))[0]
        velocity = struct.unpack('<f', bytes(rx_data[4:8]))[0]
        print(f'  Position: {position} rev, Velocity: {velocity} rev/s')
        return position, velocity
    else:
        print('  Failed to get encoder estimates')
        return None, None

# Get heartbeat message from the motor
def get_heartbeat(bus, motor_id):
    """
    Get heartbeat message from the motor
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        
    Returns:
        tuple: (axis_error, axis_state, motor_flag, encoder_flag, controller_flag)
    """
    cmd_id = 0x001
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Requesting heartbeat for motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Empty data for request
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    state, rx_data = send_can_frame(bus, arbitration_id, data)
    
    if state == 0 and len(rx_data) >= 8:
        axis_error = struct.unpack('<I', bytes(rx_data[0:4]))[0]
        axis_state = rx_data[4]
        motor_flag = rx_data[5]
        encoder_flag = rx_data[6]
        controller_flag = rx_data[7]
        print(f'  Axis error: {axis_error}, Axis state: {axis_state}, Motor flag: {motor_flag}, Encoder flag: {encoder_flag}, Controller flag: {controller_flag}')
        return axis_error, axis_state, motor_flag, encoder_flag, controller_flag
    else:
        print('  Failed to get heartbeat')
        return None, None, None, None, None

# Send Estop command to the motor
def estop(bus, motor_id):
    """
    Send Estop command to the motor
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
    """
    cmd_id = 0x002
    arbitration_id = (motor_id << 5) + cmd_id
    print(f'Sending Estop to motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Empty data for Estop command
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    print(f'  Sending data: {data}')
    send_can_frame(bus, arbitration_id, data, block_receive=0)


def get_encoder_estimates(bus, motor_id):
    """
    Get encoder position and velocity estimates
    
    Args:
        bus: CAN bus object
        motor_id: Motor ID
        
    Returns:
        tuple: (position, velocity) estimates
            - position: Encoder position in revolutions (float)
            - velocity: Encoder velocity in revolutions per second (float)
    """
    cmd_id = 0x009  # CMD ID for Get_Encoder_Estimates
    arbitration_id = (motor_id << 5) + cmd_id  # Calculate CAN ID
    print(f'Requesting encoder estimates for motor {motor_id}')
    print(f'  CAN-ID: {hex(arbitration_id)}')
    
    # Empty data for request
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    # Send CAN frame and wait for response
    state, rx_data = send_can_frame(bus, arbitration_id, data)
    
    if state == 0 and len(rx_data) >= 8:
        # Convert bytes to floats (little endian)
        position = struct.unpack('<f', bytes(rx_data[0:4]))[0]
        velocity = struct.unpack('<f', bytes(rx_data[4:8]))[0]
        print(f'  Position: {position} rev, Velocity: {velocity} rev/s')
        return position, velocity
    else:
        print('  Failed to get encoder estimates')
        return None, None



# Main function with command line arguments
def main():
    parser = argparse.ArgumentParser(description='STW Motor Control via CAN')
    parser.add_argument('--can', type=str, default='can0', help='CAN interface (default: can0)')
    parser.add_argument('--bitrate', type=int, default=500000, help='CAN bitrate (default: 500000)')
    parser.add_argument('--motor_id', type=int, default=1, help='Motor ID (default: 1)')
    parser.add_argument('--command', type=str, required=True, 
                        choices=['calibrate', 'position', 'velocity', 'status'],
                        help='Command: calibrate, position, velocity, or status')
    parser.add_argument('--value', type=float, default=0, 
                        help='Target value (position in rev or velocity in rev/s)')
    parser.add_argument('--vel_ff', type=float, default=0, 
                        help='Velocity feedforward for position control (0.001 rev/s)')
    parser.add_argument('--torque_ff', type=float, default=0, 
                        help='Torque feedforward (0.001 Nm)')
    
    args = parser.parse_args()
    
    try:
        # Initialize CAN bus
        bus = can.interface.Bus(interface='socketcan', 
                                channel=args.can,
                                bitrate=args.bitrate)
        
        print(f"Connected to {args.can} at {args.bitrate} bps")
        
        # Process commands
        if args.command == 'calibrate':
            calibrate_motor(bus, args.motor_id)
            
        elif args.command == 'position':
            # Set controller to position mode
            set_controller_mode(bus, args.motor_id, control_mode=3, input_mode=3)
            time.sleep(0.5)
            
            # Set to closed loop state
            set_closed_loop_state(bus, args.motor_id)
            time.sleep(0.5)
            
            # Set position
            set_position(bus, args.motor_id, args.value, args.vel_ff, args.torque_ff)
            
        elif args.command == 'velocity':
            # Set controller to velocity mode
            set_controller_mode(bus, args.motor_id, control_mode=2, input_mode=2)
            time.sleep(0.5)
            
            # Set to closed loop state
            set_closed_loop_state(bus, args.motor_id)
            time.sleep(0.5)
            
            # Set velocity
            set_velocity(bus, args.motor_id, args.value, args.torque_ff)
            
        elif args.command == 'status':
            # Just get encoder estimates
            get_encoder_estimates(bus, args.motor_id)
            
        # Wait a bit before shutting down
        time.sleep(1.0)
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'bus' in locals():
            # Shut down the bus
            bus.shutdown()
            print("CAN bus shut down")



def positionControlTest(bus, motorID, position):
    # # position control test success
    set_controller_mode(bus, motorID, control_mode=3, input_mode=3)
    time.sleep(0.5)
    # Set to closed loop state
    set_closed_loop_state(bus, motorID)
    time.sleep(0.5)
    # Set position
    set_position(bus, motorID, position)


def speedControlTest(bus, motor_id, speed):
    set_controller_mode(bus, motor_id, control_mode=2, input_mode=2)
    time.sleep(0.5)
    
    # Set to closed loop state
    set_closed_loop_state(bus, motor_id)
    time.sleep(0.5)
    
    # Set velocity
    set_velocity(bus, motor_id, speed, 0)
            

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
    # positionControlTest(bus, motorID5, 0)

    # speedLoopTest(bus, motorID1) pass
    # speedLoopTest(bus, motorID2) pass
    # speedLoopTest(bus, motorID3) pass
    # speedLoopTest(bus, motorID4) pass
    # speedLoopTest(bus, motorID5) 


    # err = get_error(bus, motorID5, 0)
    # print(err)

    # reboot_motor(bus, motorID5)

    # 在退出时确保资源被释放  
    bus.shutdown()


if __name__ == "__main__":
    main()
    # runMotorTest()