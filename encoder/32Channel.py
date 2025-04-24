


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
    ser = serial.Serial('COM10', 115200, timeout=1)
    
    try:
        for command in commands:
            ser.write(bytes([command]))
            response = read_serial_data(ser, 66)  # 32 channels
            # response = read_serial_data(ser, 22)  # 16 channel

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


def schedule_task(interval):
    try:
        while True:
            percentages = process_serial_commands()
            a1Pos = percentages[6]
            print(a1Pos)
            # print(percentage                                                                   s)

            time.sleep(interval)

    except KeyboardInterrupt:
        print("Exiting...")

# Start the task scheduling every 5 seconds
schedule_task(0.1)
