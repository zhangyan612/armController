# import serial

# def hex_to_percentage(hex_data):
#     percentages = []
#     for i in range(0, len(hex_data), 4):
#         high_byte = int(hex_data[i:i+2], 16)
#         low_byte = int(hex_data[i+2:i+4], 16)
#         value = (high_byte << 8) + low_byte
#         percentage = value / 100  # Further divide by 100
#         percentages.append(percentage)
#     return percentages

# # Function to handle data reading robustly
# def read_serial_data(ser, expected_bytes):
#     data = b''
#     while len(data) < expected_bytes:
#         data += ser.read(expected_bytes - len(data))
#     return data.hex()

# # Set up serial connection
# ser = serial.Serial('COM15', 115200, timeout=1)

# commands = [0xEE, 0xFF]

# try:
#     for command in commands:
#         ser.write(bytes([command]))
#         response = read_serial_data(ser, 66)  # Read the expected number of bytes
#         print(response)
#         if command == 0xEE:
#             percentages = hex_to_percentage(response[2:-2])  # Remove start and end markers
#             print(f"Decoded Percentages for command {hex(command)}: {percentages}")
#         # elif command == 0xFF:
#         #     voltages = hex_to_percentage(response[2:-2])  # Assume same processing for example
#         #     print(f"Decoded Voltages for command {hex(command)}: {voltages}")
# except KeyboardInterrupt:
#     print("Exiting...")
# finally:
#     ser.close()




# def hex_to_percentage(hex_data):
#     percentages = []
#     for i in range(0, len(hex_data), 4):
#         high_byte = int(hex_data[i:i+2], 16)
#         low_byte = int(hex_data[i+2:i+4], 16)
#         value = (high_byte << 8) + low_byte
#         percentage = value / 100  # Further divide by 100
#         percentages.append(percentage)
#     return percentages

# # Sample hex response
# hex_response = "EE0F7F0F810F810F7D0F700F6A0F690F6D0F760F680F740F750F5D0F640F670F5E0F530F580F590F5E0F570F5C0F4F0F4A0F500F4F0F540F510F5D0F570F5526CBEE"

# # Remove start and end markers
# cleaned_hex_response = hex_response[2:-2]

# percentages = hex_to_percentage(cleaned_hex_response)
# print("Percentages:", percentages)




import serial
import time
import positionMapper

def hex_to_percentage(hex_data):
    percentages = []
    for i in range(0, len(hex_data), 4):
        high_byte = int(hex_data[i:i+2], 16)
        low_byte = int(hex_data[i+2:i+4], 16)
        value = (high_byte << 8) + low_byte
        percentage = value / 100  # Further divide by 100
        percentages.append(percentage)
    return percentages

def read_serial_data(ser, expected_bytes):
    data = b''
    while len(data) < expected_bytes:
        data += ser.read(expected_bytes - len(data))
    return data.hex()

def process_serial_commands():
    commands = [0xEE, 0xFF]
    ser = serial.Serial('COM15', 115200, timeout=1)
    
    try:
        for command in commands:
            ser.write(bytes([command]))
            response = read_serial_data(ser, 66)  # Read the expected number of bytes
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


import odrive
odrv0 = odrive.find_any()

odrv0.axis0.controller.config.control_mode=odrive.utils.ControlMode.POSITION_CONTROL
odrv0.axis0.controller.config.input_mode=odrive.utils.InputMode.POS_FILTER
odrv0.axis0.requested_state=odrive.utils.AxisState.CLOSED_LOOP_CONTROL

def schedule_task(interval):
    try:
        while True:
            percentages = process_serial_commands()
            a1Pos = percentages[0]
            motor_pos = positionMapper.map_serial_to_motor(a1Pos, 0, 99.9,  -12, 11)
            # print(motor_pos)
            odrv0.axis0.controller.input_pos=motor_pos

            time.sleep(interval)
    except KeyboardInterrupt:
        print("Exiting...")

# Start the task scheduling every 5 seconds
schedule_task(0.1)

# range of motion 0-99

# success