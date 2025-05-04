


import serial
import time

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
    ser = serial.Serial('COM9', 115200, timeout=1)
    
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

def restrict_range(value, lower=-2, upper=2):
    """Restricts a number within the range [-2, 2]."""
    return max(lower, min(value, upper))

def map_to_motor(value, in_min=0, in_max=9940, out_min=-3.14, out_max=3.14):
    """Maps a value from one range to another."""
    # 计算value在in_min和in_max之间的比例
    # 然后将这个比例应用到out_min和out_max之间
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def schedule_task(interval):
    try:
        while True:
            percentages = process_serial_commands()
            a1Pos = percentages  #[0:6]
            print(a1Pos)
            # mapped_a1Pos = [map_to_motor(value) for value in a1Pos]

            # for i in [1, 2, 4]:
            #     mapped_a1Pos[i] = restrict_range(mapped_a1Pos[i])

            # print(mapped_a1Pos)

            # print(percentages)

            time.sleep(interval)

    except KeyboardInterrupt:
        print("Exiting...")

# Start the task scheduling every 5 seconds
schedule_task(0.1)
