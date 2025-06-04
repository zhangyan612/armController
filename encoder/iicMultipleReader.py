import serial
import json

# Open COM9 at 115200 baud
ser = serial.Serial('COM9', 115200, timeout=1)

while True:
    try:
        line = ser.readline().decode().strip()
        if not line:
            continue

        angles = json.loads(line)  # Parse JSON array of floats

        # Extract encoder 1 and encoder 5
        angle_1 = angles[0]  # Encoder at channel 1
        angle_5 = angles[4]  # Encoder at channel 5

        print(f"Encoder 1: {angle_1:.2f} deg | Encoder 5: {angle_5:.2f} deg")

    except (json.JSONDecodeError, IndexError, ValueError):
        continue  # Skip malformed lines
