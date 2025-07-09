import serial
import time

# Change COM port to match your system (e.g., COM3 on Windows or /dev/ttyUSB0 on Linux/Mac)
ser = serial.Serial('COM21', 115200, timeout=1)

time.sleep(2)  # Wait for Arduino to reset

while True:
    line = ser.readline().decode().strip()
    if line:
        try:
            angle = float(line)
            print(f"Angle: {angle:.2f}°")
        except ValueError:
            pass  # skip malformed lines
