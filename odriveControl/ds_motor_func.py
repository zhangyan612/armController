import can
import time
import struct

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