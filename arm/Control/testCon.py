import serial

try:
    ser = serial.Serial('/dev/arm1', baudrate=19200, timeout=0.5)
    print(f"Port {ser.name} opened successfully!")
    ser.close()
except Exception as e:
    print(f"Error opening port: {e}")