import serial
import struct
import sys
import time

# Usage: edit COM_PORT to your Windows COM port (e.g. 'COM3') or pass as first arg
COM_PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM6'
BAUDRATE = 115200

FRAME_START = 0x02
FRAME_ID = 0x08


def int16_from_bytes(low, high):
    val = (high << 8) | low
    # convert to signed 16-bit
    if val & 0x8000:
        val = val - 0x10000
    return val


def main():
    print(f"Opening {COM_PORT} @ {BAUDRATE}")
    try:
        ser = serial.Serial(COM_PORT, BAUDRATE, timeout=1)
    except Exception as e:
        print("Failed to open serial port:", e)
        return

    buf = bytearray()
    try:
        while True:
            b = ser.read(1)
            if not b:
                continue
            buf += b
            # trim until start
            while len(buf) and buf[0] != FRAME_START:
                buf.pop(0)
            if len(buf) >= 5:
                if buf[0] == FRAME_START and buf[1] == FRAME_ID:
                    d0 = buf[2]
                    d1 = buf[3]
                    chk = buf[4]
                    calc = FRAME_START ^ FRAME_ID ^ d0 ^ d1
                    if chk == calc:
                        raw_angle = int16_from_bytes(d0, d1)
                        mgt_angle = -((d0 & 0xFF) + ((d1 & 0xFF) << 8))
                        # Only print valid frames
                        print(f"FRAME,{mgt_angle},{raw_angle}")
                    # consume frame
                    del buf[:5]
                else:
                    buf.pop(0)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()


if __name__ == '__main__':
    main()
