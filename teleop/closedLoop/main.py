import time
import platform
import mqtt_receiver
import can
from pcan_cybergearLib import CANMotorController


motor_ranges = {
    1: [-1.5, 2.5],
    2: [-0.5, 2.0],
    3: [-2.0, 0.5],
    4: [-1.1, 1.4]
}

# map value from encoder reading to motor
def map_to_motor(value, in_min=0, in_max=9940, out_min=0, out_max=6.28):
    """Maps a value from one range to another."""
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# restrict the range of motion by setting joint limit
def restrict_range(value, lower=-2, upper=2):
    """Restricts a number within the range [-2, 2]."""
    return max(lower, min(value, upper))


def init_robstride_motor(bus, motor_id):
    try:
        motor = CANMotorController(bus, motor_id=motor_id, main_can_id=0)
        
        if platform.system() == "Windows":
            print(f"✓ Initializing Motor ID {motor_id} via PCAN")
            # For Windows/PCAN, we'll use a simplified initialization
            response = motor.write_single_param("run_mode", 0)
            if response and response[0] is not None:
                print(f"✓ PCAN Communication successful with Motor ID {motor_id}")
                print(f"  Position: {response[1]:.3f} rad, Velocity: {response[2]:.3f} rad/s")
                print(f"  Torque: {response[3]:.3f} Nm, Temperature: {response[4]:.1f} °C")
            else:
                # If no response, assume simulation or different protocol
                print(f"✓ Motor ID {motor_id} configured (assuming simulation)")
        else:
            # Linux with SocketCAN
            print(f"Attempting to read 'run_mode' parameter for Motor ID {motor_id}...")
            response = motor.write_single_param("run_mode", 0)
            if response and response[0] is not None:
                print(f"✓ Communication successful with Motor ID {motor_id}")
                print(f"  Position: {response[1]:.3f} rad, Velocity: {response[2]:.3f} rad/s")
                print(f"  Torque: {response[3]:.3f} Nm, Temperature: {response[4]:.1f} °C")

        # Common configuration for both platforms
        motor.write_single_param("run_mode", motor.RunModes.POSITION_MODE.value)
        print("✓ Set to POSITION_MODE")
        
        motor.write_single_param("limit_spd", 5.0)
        motor.write_single_param("loc_kp", 20.0)
        
        motor.enable()
        print("✓ Motor enabled")
        return motor
        
    except Exception as e:
        print(f"✗ Failed to initialize Motor ID {motor_id}: {e}")
        return None


class MotorController:
    def __init__(self):
        self.motors = {}
        self.current_positions = {}
        self.mqtt_receiver = None
        
    def setup_motors(self, bus):
        """Initialize all motors"""
        motor_ids = [1, 2, 3, 4]
        for motor_id in motor_ids:
            motor = init_robstride_motor(bus, motor_id)
            if motor:
                self.motors[motor_id] = motor
                self.current_positions[motor_id] = 0.0

                #if motor position is not in acceptable range, throw error
                
        print(f"✓ Successfully initialized {len(self.motors)} motors")
        
    def setup_mqtt(self):
        """Setup MQTT receiver with callback"""
        self.mqtt_receiver = mqtt_receiver.MQTTReceiver()
        self.mqtt_receiver.set_message_callback(self.handle_mqtt_message)
        
    def handle_mqtt_message(self, pot_readings):
        """Handle incoming MQTT messages and control motors"""
        if not isinstance(pot_readings, list) or len(pot_readings) < 4:
            print(f"Invalid message format: {pot_readings}")
            return
            
        # print(f"Processing potentiometer readings: {pot_readings}")
        
        # Map each potentiometer reading to motor position
        for motor_id, m_range in motor_ranges.items():
            pot_index = motor_id - 1  # assuming pot_readings[0] -> motor 1, etc.
            if pot_readings[pot_index]:
                mapped_position = map_to_motor(
                    pot_readings[pot_index], 0, 9999,
                    out_min=m_range[0], out_max=m_range[1]
                )
                print(f"Moving Motor {motor_id} to position: {mapped_position:.3f} rad")
                # restricted_position = restrict_range(mapped_position)
                # self.move_motor(motor_id, mapped_position)


    def move_motor(self, motor_id, position):
        """Move specific motor to position"""
        try:
            motor = self.motors[motor_id]
            motor.write_single_param("loc_ref", position)
            self.current_positions[motor_id] = position
            print(f"Motor {motor_id} moved to position: {position:.3f} rad")
        except Exception as e:
            print(f"Error moving motor {motor_id}: {e}")
            
    def get_motor_status(self, motor_id):
        """Get current status of motor"""
        if motor_id in self.motors:
            try:
                motor = self.motors[motor_id]
                response = motor.write_single_param("run_mode", 0)
                if response:
                    return {
                        'position': response[1],
                        'velocity': response[2],
                        'torque': response[3],
                        'temperature': response[4]
                    }
            except Exception as e:
                # If reading fails, return the last commanded position
                return {
                    'position': self.current_positions[motor_id],
                    'velocity': 0.0,
                    'torque': 0.0,
                    'temperature': 25.0
                }
        return None


def create_can_bus():
    """Create CAN bus interface based on platform"""
    if platform.system() == "Windows":
        # Use PCAN on Windows
        try:
            print("Initializing PCAN interface...")
            # Common PCAN channels: 'PCAN_USBBUS1', 'PCAN_PCIBUS1', 'PCAN_ISABUS1', etc.
            bus = can.Bus(interface='pcan', channel='PCAN_USBBUS1', bitrate=1000000)
            print("✓ PCAN bus initialized successfully")
            return bus
        except Exception as e:
            print(f"Failed to initialize PCAN: {e}")
            print("Available PCAN channels: PCAN_USBBUS1, PCAN_USBBUS2, PCAN_PCIBUS1, etc.")
            raise
    else:
        # Use SocketCAN on Linux
        print("Initializing SocketCAN interface...")
        bus = can.Bus(interface="socketcan", channel="can0", bitrate=1000000)
        print("✓ SocketCAN bus initialized successfully")
        return bus


def main():
    """Main function demonstrating how to use both components"""
    print("Starting MQTT-driven RobStride motor control system...")
    print(f"Platform: {platform.system()}")
    
    # Create motor controller
    controller = MotorController()
    
    try:
        # Create CAN bus
        robstride_bus = create_can_bus()
        
        # # Initialize motors
        controller.setup_motors(robstride_bus)
        
        if not controller.motors:
            print("No motors initialized. Continuing with MQTT for testing...")
            
        # Setup MQTT
        controller.setup_mqtt()
        
        # Start MQTT receiver
        controller.mqtt_receiver.start()

        # Keep program running
        print("System running. Press Ctrl+C to stop.")
        print("Motor mapping: Potentiometer 0 -> Motor 1, Potentiometer 1 -> Motor 2, etc.")
        print("Waiting for MQTT messages...")
        
        # Keep the main thread alive forever, waiting for MQTT messages
        # The MQTT receiver runs in its own thread and will process messages continuously
        while True:
            time.sleep(0.1)  # Minimal sleep to reduce CPU usage
            
    except KeyboardInterrupt:
        print("\nStopping system...")
    except Exception as e:
        print(f"System error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if controller.mqtt_receiver:
            controller.mqtt_receiver.stop()
        # if 'robstride_bus' in locals():
            # try:
            #     # robstride_bus.shutdown()
            # except:
            #     pass
        print("System stopped completely.")

if __name__ == "__main__":
    main()