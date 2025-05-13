


import numpy as np
import serial
import time
from Actutor import Robot
from Motor485MultiDevice import MotorControl

def hex_to_percentage(hex_data):
    percentages = []
    for i in range(0, len(hex_data), 4):
        high_byte = int(hex_data[i:i+2], 16)
        low_byte = int(hex_data[i+2:i+4], 16)
        value = (high_byte << 8) + low_byte
        percentage = value  # Further divide by 100 / 100 
        percentages.append(percentage)
    return percentages

def read_serial_data(ser, expected_bytes):
    data = b''
    while len(data) < expected_bytes:
        data += ser.read(expected_bytes - len(data))
    return data.hex()

def process_serial_commands():
    commands = [0xEE, 0xFF]
    ser = serial.Serial('COM10', 115200, timeout=1)
    
    try:
        for command in commands:
            ser.write(bytes([command]))
            # response = read_serial_data(ser, 66)  # 32 channels
            response = read_serial_data(ser, 22)  # 16 channel

            if command == 0xEE:
                percentages = hex_to_percentage(response[2:-2])  # Remove start and end markers
                return percentages
                # print(f"Decoded Percentages for command {hex(command)}: {percentages}")
            # elif command == 0xFF:
            #     voltages = hex_to_percentage(response[2:-2])  # Assume same processing for example
            #     print(f"Decoded Voltages for command {hex(command)}: {voltages}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()


def map_to_motor(value, in_min=0, in_max=9985, out_min=0, out_max=1671168):
    """Maps a value from one range to another."""
    # 计算value在in_min和in_max之间的比例
    # 然后将这个比例应用到out_min和out_max之间
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

def schedule_task(interval):
    # robot = Robot("COM8", 2250000)
    # # 设置电机ID与减速比
    # # robot.register_motor(2, 101)
    # robot.register_motor(1, 101)
    # robot.register_motor(2, 101)
    # robot.register_motor(3, 101)
    # robot.register_motor(4, 101)
    # robot.register_motor(5, 101)
    # robot.register_motor(6, 101)

    # vel = np.array([2.5, 2.5, 2.5, 2.5, 2.5, 2.5])
    # robot.servo_enable()

    # 单圈 32768 x 10 x 51  
    # 51 速比 单圈 1671168
    mc = MotorControl(port='COM8', baudrate=19200)
    print("Version Info:", mc.version_info())
    print("Temperature:", mc.read_temperature())
    print("Servo Status:", mc.servo_status())
    print("Position1:", mc.position1())
    print("Position2:", mc.position2())

    print("Disabling servo...")
    mc.disable()
    print("Servo Status after disable:", mc.servo_status())
    print("Enabling servo...")
    mc.enable()

    print("Setting target speed to 1000 RPM...", mc.set_target_speed(1000))
    print("Setting accel to 500 ms...", mc.set_accel(500))
    print("Setting decel to 500 ms...", mc.set_decel(500))

    try:
        while True:
            percentages = process_serial_commands()
            a1Pos = percentages[0:6]
            # print(a1Pos)
            mapped_a1Pos = [map_to_motor(value) for value in a1Pos]
            print(mapped_a1Pos[0])
            target_pos = mapped_a1Pos[0]
            # pos1 = np.array([mapped_a1Pos])
            # print(mapped_a1Pos[2])
            current_pos = mc.position1()
            if abs(target_pos - current_pos) > 1000:
                print("Moving with accel to {mapped_a1Pos[0]}", mc.move_accel(mapped_a1Pos[0]))

            # robot.write_position(mapped_a1Pos, vel)

            # print(percentages)
            time.sleep(interval)

    except KeyboardInterrupt:
        print("Exiting...")
        mc.disable()

# Start the task scheduling every 5 seconds
schedule_task(0.1)
