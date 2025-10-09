import serial

for i in range(8):
    port = f"/dev/ttyCH9344USB{i}"
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        print(f"Opened {port}")
        ser.close()
    except Exception as e:
        print(f"Failed to open {port}: {e}")
