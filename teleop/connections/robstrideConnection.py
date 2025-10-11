from pcan_cybergearLib import CANMotorController
import can
import time

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

            # print(motor.set_0_pos())

        else:
            print(f"✗ No response from Motor ID {motor_id} — check connection or ID")

    except Exception as e:
        print(f"✗ Error communicating with Motor ID {motor_id}: {e}")


def move_motor(motor, position):
    try:
        motor.write_single_param("loc_ref", position)
        print(f"Motor moved to position: {position:.3f} rad")
    except Exception as e:
        print(f"✗ Error moving motor: {e}")

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

        # test_motor_connection(bus, 4)


        # motor = CANMotorController(bus, motor_id=2, main_can_id=0)
        # motor.write_single_param("run_mode", motor.RunModes.POSITION_MODE.value)

        # # print("✓ Set to POSITION_MODE")

        # # # Set position control parameters
        # motor.write_single_param("limit_spd", 5.0)  # Limit speed to 5 rad/s
        # motor.write_single_param("loc_kp", 20.0)    # Position gain

        # motor.enable()

        # time.sleep(1)

        # motor.write_single_param("loc_ref", 0.3)

        time.sleep(2)
        # motor.disable()


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


# motor 1 [-1.5, 2.5]

# motor 2  [-0.5, 2]

# motor 3  [-2, 0.5]

# motor 4  [-1.1, 1.4]