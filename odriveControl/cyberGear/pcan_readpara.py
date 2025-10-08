from pcan_cybergearLib import CANMotorController
import can


def test_motor_connection(bus, motor_id):
    print(f"\n=== Testing Motor ID {motor_id} ===")
    try:
        motor = CANMotorController(bus, motor_id=motor_id, main_can_id=0)

        # Try to read run_mode to check if communication works
        print("Attempting to read 'run_mode' parameter...")
        response = motor.write_single_param("run_mode", 0)
        if response and response[0] is not None:
            print(f"✓ Communication successful with Motor ID {motor_id}")
            print(f"  Position: {response[1]:.3f} rad, Velocity: {response[2]:.3f} rad/s")
            print(f"  Torque: {response[3]:.3f} Nm, Temperature: {response[4]:.1f} °C")

            # Optionally read more parameters
            parameters_to_test = [
                "mechPos",
                "mechVel",
                "VBUS",
                "rotation",
                "iqf",
            ]
            for param in parameters_to_test:
                try:
                    value = motor.read_single_param(param)
                    print(f"  {param}: {value}")
                except Exception as e:
                    print(f"  {param}: Error - {e}")
        else:
            print(f"✗ No response from Motor ID {motor_id} — check connection or ID")

    except Exception as e:
        print(f"✗ Error communicating with Motor ID {motor_id}: {e}")


# sudo ip link set down can0
# sudo ip link set can0 type can bitrate 1000000 loopback off
# sudo ip link set up can0

if __name__ == "__main__":
    try:
        # Initialize CAN bus
        bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
        print("CAN bus initialized successfully")

        # Test motors 1–4
        for motor_id in range(1, 5):
            test_motor_connection(bus, motor_id)
        # test_motor_connection(bus, 6)
        
    except can.CanError as e:
        print(f"CAN error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            bus.shutdown()
            print("\nCAN bus shutdown")
        except:
            pass
