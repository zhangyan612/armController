frame_id = 0x1E
print(hex(frame_id))


hex_value = "1E"  # This is the hexadecimal value
number = int(hex_value, 16)  # Convert hex to an integer
print(number)  # Output will be 30


def format_can_id(can_id):
    # Convert the integer to a hexadecimal string, zero-padded to 8 characters
    hex_str = f"{can_id:08X}"
    # Split the string into 2-character chunks and join them with spaces
    formatted_id = ' '.join(hex_str[i:i+2] for i in range(0, len(hex_str), 2))
    return formatted_id

# Example usage
can_id = 0x01E
formatted_id = format_can_id(can_id)
print(formatted_id)  # Output: "18 FF 00 23"