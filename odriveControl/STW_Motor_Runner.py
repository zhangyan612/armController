import can
import time
import struct
import platform

# CAN bus initialization with platform detection
def init_can_bus():
    if platform.system() == 'Windows':
        interface = 'pcan'
        channel = 'PCAN_USBBUS1'
        bitrate = 500000
    else:
        interface = 'socketcan'
        channel = 'can0'
        bitrate = 500000
    
    try:
        return can.interface.Bus(
            interface=interface,
            channel=channel,
            bitrate=bitrate
        )
    except Exception as e:
        print(f"CAN bus initialization failed: {e}")
        return None

# Motor control functions
def set_controller_mode(bus, motor_id, control_mode, input_mode):
    cmd_id = 0x0B
    arbitration_id = (motor_id << 5) + cmd_id
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
    message = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False
    )
    bus.send(message)

def set_closed_loop_state(bus, motor_id):
    cmd_id = 0x07
    arbitration_id = (motor_id << 5) + cmd_id
    data = [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False
    )
    bus.send(message)

def set_position(bus, motor_id, position):
    cmd_id = 0x0C
    arbitration_id = (motor_id << 5) + cmd_id
    pos_bytes = struct.pack('<f', position)
    # Default feedforwards to 0
    vel_ff_bytes = struct.pack('<h', 0)
    torque_ff_bytes = struct.pack('<h', 0)
    data = list(pos_bytes) + list(vel_ff_bytes) + list(torque_ff_bytes)
    message = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False
    )
    bus.send(message)

def get_encoder_estimates(bus, motor_id):
    cmd_id = 0x009
    arbitration_id = (motor_id << 5) + cmd_id
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    message = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False
    )
    bus.send(message)
    
    # Wait for response
    start_time = time.time()
    while time.time() - start_time < 0.5:
        msg = bus.recv(0.1)
        if msg and msg.arbitration_id == arbitration_id:
            if len(msg.data) >= 8:
                position = struct.unpack('<f', bytes(msg.data[0:4]))[0]
                velocity = struct.unpack('<f', bytes(msg.data[4:8]))[0]
                return position, velocity
    return None, None

def set_axis_state(bus, motor_id, state):
    cmd_id = 0x007
    arbitration_id = (motor_id << 5) + cmd_id
    state_bytes = struct.pack('<I', state)
    data = list(state_bytes) + [0x00, 0x00, 0x00, 0x00]
    message = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False
    )
    bus.send(message)

def initialize_motors(bus, motor_ids):
    """Initialize motors to closed-loop position mode"""
    for motor_id in motor_ids:
        set_controller_mode(bus, motor_id, 3, 3)  # Position mode
        time.sleep(0.1)
        set_closed_loop_state(bus, motor_id)
        time.sleep(0.1)
    print(f"Initialized motors {motor_ids} in position mode")

def get_all_positions(bus, motor_ids):
    """Get current positions for all motors"""
    positions = {}
    for motor_id in motor_ids:
        pos, _ = get_encoder_estimates(bus, motor_id)
        positions[motor_id] = pos
        time.sleep(0.05)  # Short delay between queries
    return positions

def move_motor(bus, motor_id, target_position):
    """Move motor to specified position and release"""
    # Get current position for reference
    current_pos, _ = get_encoder_estimates(bus, motor_id)
    print(f"Moving motor {motor_id} from {current_pos:.3f} to {target_position:.3f}")
    
    # Move to target
    set_position(bus, motor_id, target_position)
    time.sleep(2.0)  # Wait for movement
    
    # Release motor
    set_axis_state(bus, motor_id, 1)  # AXIS_STATE_IDLE
    print(f"Motor {motor_id} released")

def main_control_loop():
    bus = init_can_bus()
    if not bus:
        return
    
    # Define motors to control (1-6)
    motor_ids = [1, 2, 3, 4, 5, 6]
    
    try:
        # Initialize all motors
        initialize_motors(bus, motor_ids)
        
        # Get and display initial positions
        positions = get_all_positions(bus, motor_ids)
        print("\nCurrent motor positions:")
        for motor_id, pos in positions.items():
            print(f"  Motor {motor_id}: {pos:.3f} rev")
        
        # Interactive control loop
        while True:
            try:
                # Prompt for input
                command = input("\nEnter command (motor_id position) or 'q' to quit: ").strip()
                
                if command.lower() in ['q', 'quit', 'exit']:
                    break
                
                # Parse input
                parts = command.split()
                if len(parts) != 2:
                    raise ValueError("Invalid format. Use: <motor_id> <position>")
                
                motor_id = int(parts[0])
                target_position = float(parts[1])
                
                if motor_id not in motor_ids:
                    print(f"Error: Motor {motor_id} not in controlled motors")
                    continue
                
                # Move the motor
                move_motor(bus, motor_id, target_position)
                
                # Get updated positions
                positions = get_all_positions(bus, motor_ids)
                print("\nUpdated motor positions:")
                for mid, pos in positions.items():
                    print(f"  Motor {mid}: {pos:.3f} rev")
                    
            except ValueError as e:
                print(f"Error: {e}")
            except Exception as e:
                print(f"Unexpected error: {e}")
                
    finally:
        # Clean up
        for motor_id in motor_ids:
            try:
                set_axis_state(bus, motor_id, 1)  # Ensure idle state
            except:
                pass
        bus.shutdown()
        print("\nMotors released. CAN bus shut down.")

if __name__ == "__main__":
    main_control_loop()