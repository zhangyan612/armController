from pcan_cybergear import CANMotorController
import time
import can

if __name__ == "__main__":

    try:
        # Initialize CAN bus
        bus = can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
        print("CAN bus initialized successfully")
        
        # Create motor controller instance
        motor = CANMotorController(bus, motor_id=6, main_can_id=0)
        
        print("=== Testing Parameter Reading First ===")
        
        # Test reading some parameters to verify communication
        try:
            print("\n1. Testing parameter reading...")
            
            # Try to read run_mode first
            print("Attempting to read run_mode parameter...")
            
            # We'll use the write method to see if we get a response (even though we're not writing)
            # This will help verify communication
            response = motor.write_single_param("run_mode", 0)
            if response and response[0] is not None:
                print(f"✓ Communication successful! Motor ID: {response[0]}")
                print(f"  Position: {response[1]:.3f} rad, Velocity: {response[2]:.3f} rad/s")
                print(f"  Torque: {response[3]:.3f} Nm, Temperature: {response[4]:.1f} °C")
            else:
                print("✗ No response from motor - check connection and motor ID")
                
                
        except Exception as e:
            print(f"✗ Error reading parameters: {e}")
            

        print("\n=== Testing All 4 Modes ===")

        # Test 1: CONTROL_MODE (运控模式)
        print("\n1. Testing CONTROL_MODE (运控模式)...")
        try:
            motor.write_single_param("run_mode", motor.RunModes.CONTROL_MODE.value)
            print("✓ Set to CONTROL_MODE")
            
            motor.enable()
            print("✓ Motor enabled")
            
            # Send control commands
            print("Sending control commands...")
            for i in range(5):
                torque = 0.5 * (1 if i % 2 == 0 else -1)  # Alternate torque
                response = motor.send_motor_control_command(
                    torque=torque,
                    target_angle=1.0,
                    target_velocity=2.0,
                    Kp=10,
                    Kd=0.5
                )
                if response and response[0] is not None:
                    print(f"  Command {i+1}: Pos={response[1]:.3f}rad, Vel={response[2]:.3f}rad/s")
                time.sleep(0.5)
                
            motor.disable()
            print("✓ Motor disabled")
            
        except Exception as e:
            print(f"✗ Error in CONTROL_MODE: {e}")

        # Test 3: SPEED_MODE (速度模式)
        print("\n3. Testing SPEED_MODE (速度模式)...")
        try:
            motor.write_single_param("run_mode", motor.RunModes.SPEED_MODE.value)
            print("✓ Set to SPEED_MODE")
            
            motor.enable()
            print("✓ Motor enabled")
            
            # Test different speeds
            speeds = [3.0, -2.0, 5.0, 0.0]
            for speed in speeds:
                print(f"Setting speed: {speed} rad/s")
                motor.write_single_param("spd_ref", speed)
                time.sleep(3)  # Run at this speed for 3 seconds
                
                # Read current velocity
                response = motor.write_single_param("run_mode", motor.RunModes.SPEED_MODE.value)
                if response and response[0] is not None:
                    print(f"  Current velocity: {response[2]:.3f} rad/s")
            
            motor.disable()
            print("✓ Motor disabled")
            
        except Exception as e:
            print(f"✗ Error in SPEED_MODE: {e}")

        # Test 4: CURRENT_MODE (电流模式)
        print("\n4. Testing CURRENT_MODE (电流模式)...")
        try:
            motor.write_single_param("run_mode", motor.RunModes.CURRENT_MODE.value)
            print("✓ Set to CURRENT_MODE")
            
            motor.enable()
            print("✓ Motor enabled")
            
            # Test different current/torque values
            currents = [0.3, -0.2, 0.5, 0.0]
            for current in currents:
                print(f"Setting current/torque: {current} A/Nm")
                motor.write_single_param("iq_ref", current)
                time.sleep(2)  # Apply current for 2 seconds
                
                # Read current torque
                response = motor.write_single_param("run_mode", motor.RunModes.CURRENT_MODE.value)
                if response and response[0] is not None:
                    print(f"  Current torque: {response[3]:.3f} Nm")
            
            motor.disable()
            print("✓ Motor disabled")
            
        except Exception as e:
            print(f"✗ Error in CURRENT_MODE: {e}")


        # Test 2: POSITION_MODE (位置模式)
        print("\n2. Testing POSITION_MODE (位置模式)...")
        try:
            motor.write_single_param("run_mode", motor.RunModes.POSITION_MODE.value)
            print("✓ Set to POSITION_MODE")
            
            # Set position control parameters
            motor.write_single_param("limit_spd", 5.0)  # Limit speed to 5 rad/s
            motor.write_single_param("loc_kp", 20.0)    # Position gain
            
            motor.enable()
            print("✓ Motor enabled")
            
            # Move to different positions
            positions = [1.0, -1.0, 0.5, 0.0]
            for pos in positions:
                print(f"Moving to position: {pos} rad")
                motor.write_single_param("loc_ref", pos)
                time.sleep(2)  # Wait for movement
                
                # Read current position
                response = motor.write_single_param("run_mode", motor.RunModes.POSITION_MODE.value)
                if response and response[0] is not None:
                    print(f"  Current position: {response[1]:.3f} rad")
            
            motor.disable()
            print("✓ Motor disabled")
            
        except Exception as e:
            print(f"✗ Error in POSITION_MODE: {e}")


        print("\n=== All Tests Completed ===")

    except can.CanError as e:
        print(f"CAN error: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            bus.shutdown()
            print("CAN bus shutdown")
        except:
            pass