"""
pcan_auto_listener.py
- Tries to open PCAN via python-can. If not available, falls back to serial
  reading and parsing the MCU's printf lines.

Usage:
  python tools/pcan_auto_listener.py

Dependencies:
  pip install python-can pyserial

"""
import time
import sys
import re

try:
    import can
    CAN_AVAILABLE = True
except Exception:
    CAN_AVAILABLE = False

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except Exception:
    SERIAL_AVAILABLE = False

FRAME_RE = re.compile(r"CAN\s+RX\s+(STD|EXT)\s+ID=0x([0-9A-Fa-f]+)\s+DLC=(\d+)\s*(.*)")


def list_detected_configs():
    configs = []
    if CAN_AVAILABLE:
        try:
            # python-can doesn't provide a direct detect helper for PCAN; user can set channel directly
            pass
        except Exception:
            pass
    if SERIAL_AVAILABLE:
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            configs.append({"interface": "serial", "channel": p.device})
    return configs


def run_pcan(channel="PCAN_USBBUS1", bitrate=500000):
    print(f"Trying PCAN channel {channel} at {bitrate}...")
    try:
        bus = can.interface.Bus(bustype="pcan", channel=channel, bitrate=bitrate)
    except Exception as e:
        print("PCAN open failed:", e)
        return False

    print("PCAN open OK, listening...")
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
            ts = f"{msg.timestamp:.6f}" if msg.timestamp else "N/A"
            idstr = f"0x{msg.arbitration_id:08X}" if msg.is_extended_id else f"0x{msg.arbitration_id:03X}"
            ide = "EXT" if msg.is_extended_id else "STD"
            data = ' '.join(f"{b:02X}" for b in (msg.data or b""))
            print(f"[{ts}] {ide} ID={idstr} DLC={msg.dlc} DATA={data}")
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        bus.shutdown()
    return True


def run_serial(port, baud=115200):
    print(f"Opening serial {port} @ {baud}... (looking for MCU printf CAN lines)")
    try:
        s = serial.Serial(port, baudrate=baud, timeout=1.0)
    except Exception as e:
        print("Failed to open serial port:", e)
        return False

    print("Serial open OK, reading lines...")
    try:
        while True:
            line = s.readline().decode(errors='ignore').strip()
            if not line:
                continue
            m = FRAME_RE.search(line)
            if m:
                kind, idhex, dlc, rest = m.groups()
                print(f"MCU -> {kind} ID=0x{idhex} DLC={dlc} DATA={rest}")
            else:
                print("SERIAL:", line)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        s.close()
    return True


def main():
    configs = list_detected_configs()
    print("Detected configs:", configs)

    # Try PCAN first if library available
    if CAN_AVAILABLE:
        # You may want to change channel/bitrate here
        if run_pcan(channel="PCAN_USBBUS1", bitrate=500000):
            return

    # Fallback: use first detected serial port
    if configs:
        serial_cfg = next((c for c in configs if c['interface'] == 'serial'), None)
        if serial_cfg:
            run_serial(serial_cfg['channel'], baud=115200)
            return

    print("No available CAN or serial interfaces found. Install python-can and/or pyserial.")

if __name__ == '__main__':
    main()
