def map_serial_to_motor(input_value, in_min, in_max, out_min, out_max):
    # Check if input_value is outside the boundary
    if input_value < in_min or input_value > in_max:
        return None
    # Linear interpolation formula
    return (input_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == '__main__':
    # Testing the function
    serial_input = 60  # Example serial input
    motor_position = map_serial_to_motor(serial_input, 50, 100, -12, 11)
    print(f"Serial Input: {serial_input}, Mapped Motor Position: {motor_position}")