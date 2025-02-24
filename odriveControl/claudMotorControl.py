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


def runMotor():
    bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=500000) 
    print(f"Connected to can bus")

    motorID1 = 0x01                      
    calibrate_motor(bus,motorID1)             # 执行校准

    motorID1 = 0x02                       
    calibrate_motor(bus,motorID1)             # 执行校准

    # # position control
    # set_controller_mode(bus, motorID1, control_mode=3, input_mode=3)
    # time.sleep(0.5)
    # # Set to closed loop state
    # set_closed_loop_state(bus, motorID1)
    # time.sleep(0.5)
    # # Set position
    # set_position(bus, motorID1, 0)


    # 在退出时确保资源被释放  
    bus.shutdown() 




if __name__ == "__main__":
    # main()
    runMotor()