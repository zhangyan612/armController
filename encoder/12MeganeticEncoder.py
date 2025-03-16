import serial

def decode_hex_to_text(hex_data):
    # Clean up the input by removing spaces and newlines
    hex_clean = hex_data.replace("\n", "").replace(" ", "")
    
    # Decode the hex string to bytes
    bytes_data = bytes.fromhex(hex_clean)
    
    # Decode the bytes to a string, ignoring errors for non-text characters
    decoded_text = bytes_data.decode('utf-8', errors='ignore')
    
    return decoded_text

def parse_channels(decoded_text):
    # Split the text into lines
    lines = decoded_text.strip().split("\n")
    
    # Parse each line into a dictionary for channel and its value
    channels = {}
    for line in lines:
        parts = line.split()
        if len(parts) == 2:  # Ensure there are exactly two parts
            try:
                channel, value = map(int, parts)  # Convert channel and value to integers
                channels[channel] = value  # Store in a dictionary with channel as the key
            except ValueError as e:
                print(f"Error parsing line: {line}. Error: {e}")
        else:
            print(f"Skipping incomplete line: {line}")
    
    return channels

def read_and_parse_serial_data(ser, target_channels):
    """
    Read data from the serial port and parse specified channels.
    
    :param ser: Serial port object
    :param target_channels: List of channels to monitor (e.g., [10, 12, 15])
    """
    # Buffer to store received data
    data_buffer = []

    try:
        while True:
            # Read data from the serial port
            if ser.in_waiting > 0:
                hex_data = ser.read(ser.in_waiting)  # Read all available data
                # Convert hex data to readable format
                readable_data = hex_data.hex()       # Get the hex string
                decoded_text = decode_hex_to_text(readable_data)  # Decode to text
                # print(f"Received Hex: {readable_data}")
                # print(f"Decoded Text: {decoded_text}")
                
                # Append the decoded text to the buffer
                data_buffer.append(decoded_text)
                
                # Combine the buffer into a single string
                combined_data = "".join(data_buffer)
                
                # Check if the combined data contains any of the target channels
                if any(f"{channel}" in combined_data for channel in target_channels):
                    # Parse channels
                    channel_data = parse_channels(combined_data)
                    
                    # Print the values of the target channels
                    for channel in target_channels:
                        if channel in channel_data:
                            print(f"Channel {channel}: {channel_data[channel]}")
                        else:
                            print(f"Channel {channel} not found in the parsed data!")
                    
                    # Clear the buffer after processing
                    data_buffer.clear()
                else:
                    print(f"Waiting for target channels {target_channels} data...")

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()  # Close the serial port

# Configure the serial connection
ser = serial.Serial(
    port='COM9',      # Replace with your COM port
    baudrate=115200,   # Set to the baud rate used by your device
    timeout=1          # Timeout in seconds
)

# Target channels to monitor
target_channels = [1, 8, 9, 14, 15]

# Start reading and parsing data
read_and_parse_serial_data(ser, target_channels)