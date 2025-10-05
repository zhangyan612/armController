"""
Simple RS485 receiver for Windows COM5 using pyserial.
Listens on COM5 at 115200 8N1 and prints incoming data with timestamps.
"""
import serial
import time

PORT = 'COM3'
BAUD = 115200

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f'Opened {PORT} at {BAUD} bps')
    except Exception as e:
        print(f'Failed to open {PORT}: {e}')
        return

    try:
        while True:
            data = ser.readline()  # read until newline or timeout
            if data:
                ts = time.strftime('%Y-%m-%d %H:%M:%S')
                try:
                    s = data.decode('utf-8', errors='replace').rstrip('\r\n')
                except Exception:
                    s = repr(data)
                print(f'[{ts}] {s}')
            else:
                # no full line available, read any bytes
                n = ser.in_waiting
                if n:
                    raw = ser.read(n)
                    ts = time.strftime('%Y-%m-%d %H:%M:%S')
                    try:
                        s = raw.decode('utf-8', errors='replace')
                    except Exception:
                        s = repr(raw)
                    print(f'[{ts}] {s}', end='')
                else:
                    time.sleep(0.05)
    except KeyboardInterrupt:
        print('\nExiting')
    finally:
        ser.close()

if __name__ == '__main__':
    main()
