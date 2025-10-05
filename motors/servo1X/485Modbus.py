"""rs485_send.py

Small RS485 sender for Windows COM5 using pyserial.
This version explicitly toggles RTS around the transmit to assert DE on adapters
that are wired to RTS for driver-enable. It also includes an alternate path using
pyserial's RS485 helper if available.

Usage: edit PORT/BAUD/MSG and run with the Python installed on your machine.
"""
import time
try:
    import serial
except Exception as e:
    print('pyserial required: pip install pyserial')
    raise

PORT = 'COM3'
BAUD = 115200
MSG = b'C;'


def send_with_rts(port=PORT, baud=BAUD, msg=MSG, timeout=0.5, rts_settle=0.05, post_tx_wait=0.05):
    """Open serial port, assert RTS for TX, write, then clear RTS and read responses.

    This works for many USB-RS485 adapters that tie RTS to DE.
    rts_settle: seconds to wait after setting RTS before writing (adapter-dependent).
    """
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f'Opened {port} at {baud} bps')
    except Exception as e:
        print(f'Failed to open {port}: {e}')
        return

    try:
        # Some adapters drive DE when RTS=True, others ignore it. We toggle RTS
        # explicitly here to request the adapter to drive the bus.
        print('Asserting RTS (DE) for transmit...')
        ser.setRTS(True)
        time.sleep(rts_settle)  # let adapter enable driver

        print(f'Sending: {msg!r}')
        ser.write(msg)
        ser.flush()

        # Some adapters need the line to be idle before disabling DE. Wait for a
        # short time or until the UART TC flag would be set (not accessible here).
        time.sleep(post_tx_wait)

        # Clear RTS to release the bus (back to receive).
        ser.setRTS(False)
        print('Released RTS (DE). Waiting for response...')

        # read for up to 2 seconds for a reply
        deadline = time.time() + 2.0
        while time.time() < deadline:
            data = ser.readline()
            if data:
                # Print both printable attempt and raw hex + integer values for diagnosis
                try:
                    printable = data.decode('utf-8', errors='replace').rstrip('\r\n')
                except Exception:
                    printable = '<decode error>'
                hexstr = data.hex()
                ints = ','.join(str(b) for b in data)
                print(f"Received (printable): {printable!r}  hex: {hexstr}  bytes: [{ints}]")
                # keep reading until timeout
            else:
                time.sleep(0.05)

    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        try:
            ser.close()
        except Exception:
            pass


def send_with_rs485_helper(port=PORT, baud=BAUD, msg=MSG, timeout=0.5):
    """Try the pyserial RS485 wrapper if available; it can handle DE automatically.

    Not all pyserial builds include serial.rs485; this is a convenience path.
    """
    try:
        from serial.rs485 import RS485
    except Exception:
        print('serial.rs485.RS485 not available in this pyserial build')
        return

    try:
        ser = RS485(port, baudrate=baud, timeout=timeout)
        # Some adapters expect RTS high for TX; RS485() accepts parameters for that,
        # but RS485 defaults are usually OK for many adapters.
        print(f'Opened RS485 helper on {port} at {baud} bps')
    except Exception as e:
        print(f'Failed to open {port} as RS485: {e}')
        return

    try:
        print(f'Sending via RS485 helper: {msg!r}')
        ser.write(msg)
        ser.flush()
        print('Sent; reading reply...')
        deadline = time.time() + 2.0
        while time.time() < deadline:
            data = ser.readline()
            if data:
                try:
                    printable = data.decode('utf-8', errors='replace').rstrip('\r\n')
                except Exception:
                    printable = '<decode error>'
                hexstr = data.hex()
                ints = ','.join(str(b) for b in data)
                print(f"Received (printable): {printable!r}  hex: {hexstr}  bytes: [{ints}]")
            else:
                time.sleep(0.05)
    finally:
        ser.close()


def send_blocked_message(msg, port=PORT, baud=BAUD, repeats=1, timeout=0.5, rts_settle=0.05, inter_byte_delay=0.01, post_tx_wait=0.1, de_control='RTS'):
    """Assert RTS once, write message bytes one-by-one while keeping DE asserted,
    then wait and deassert. This avoids DE glitches between characters on some adapters.
    """
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f'Opened {port} at {baud} bps for blocked-message test')
    except Exception as e:
        print(f'Failed to open {port}: {e}')
        return

    try:
        for r in range(repeats):
            print(f"--> Sending blocked message (repeat {r+1}/{repeats}): {msg!r}")
            # Assert DE using selected control signal
            if de_control == 'RTS':
                ser.setRTS(True)
            elif de_control == 'DTR':
                ser.setDTR(True)
            else:
                # no explicit DE control
                pass
            time.sleep(rts_settle)
            for b in msg:
                ser.write(bytes([b]))
                ser.flush()
                time.sleep(inter_byte_delay)
            time.sleep(post_tx_wait)
            # Release DE
            if de_control == 'RTS':
                ser.setRTS(False)
            elif de_control == 'DTR':
                ser.setDTR(False)
            else:
                pass

            # read reply up to 1s
            deadline = time.time() + 1.0
            collected = bytearray()
            while time.time() < deadline:
                chunk = ser.read(64)
                if chunk:
                    collected.extend(chunk)
                else:
                    time.sleep(0.01)

            if collected:
                hexstr = collected.hex()
                ints = ','.join(str(x) for x in collected)
                try:
                    printable = collected.decode('utf-8', errors='replace')
                except Exception:
                    printable = '<decode error>'
                print(f"  Reply printable: {printable!r}  hex: {hexstr}  bytes: [{ints}]")
                # return the collected reply for caller to inspect
                return bytes(collected)
            else:
                print('  No reply received')
            time.sleep(0.05)
    finally:
        try:
            ser.close()
        except Exception:
            pass

    return None




def modbus_crc16(data: bytes) -> int:
    """Compute Modbus RTU CRC16 (returns 16-bit int where low byte is CRC low)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def send_modbus_read(slave: int, start_reg: int, count: int, **kwargs):
    """Build a Modbus RTU Read Holding Registers (0x03) request and send it.

    Uses `send_blocked_message()` to keep DE asserted for the whole frame.
    Additional kwargs are passed to `send_blocked_message` (e.g., rts_settle).
    """
    if slave < 0 or slave > 0xFF:
        raise ValueError('slave must be 0-255')
    if start_reg < 0 or start_reg > 0xFFFF:
        raise ValueError('start_reg must be 0-0xFFFF')
    if count < 1 or count > 125:
        raise ValueError('count must be 1-125')

    frame = bytearray()
    frame.append(slave & 0xFF)
    frame.append(0x03)  # function code 3 = Read Holding Registers
    frame.append((start_reg >> 8) & 0xFF)
    frame.append(start_reg & 0xFF)
    frame.append((count >> 8) & 0xFF)
    frame.append(count & 0xFF)
    crc = modbus_crc16(frame)
    frame.append(crc & 0xFF)      # CRC low
    frame.append((crc >> 8) & 0xFF)  # CRC high

    print(f'Sending Modbus READ: {frame.hex()}')
    # prefer blocked-message with RTS by default, caller may pass de_control='DTR'
    return send_blocked_message(bytes(frame), **kwargs)


def send_single_byte_test(bytes_list, port=PORT, baud=BAUD, repeats=1, timeout=0.5, rts_settle=0.05, post_tx_wait=0.05):
    """Send single bytes from bytes_list repeatedly and print the reply in hex/decimal.

    This toggles RTS for each byte to ensure DE is active only while the byte is on the bus.
    """
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f'Opened {port} at {baud} bps for single-byte tests')
    except Exception as e:
        print(f'Failed to open {port}: {e}')
        return

    try:
        for b in bytes_list:
            for i in range(repeats):
                print(f"--> Sending single byte 0x{b:02X} (repeat {i+1}/{repeats})")
                ser.setRTS(True)
                time.sleep(rts_settle)
                ser.write(bytes([b]))
                ser.flush()
                time.sleep(post_tx_wait)
                ser.setRTS(False)

                # Collect any reply bytes for up to 0.5s
                deadline = time.time() + 0.5
                collected = bytearray()
                while time.time() < deadline:
                    chunk = ser.read(64)
                    if chunk:
                        collected.extend(chunk)
                    else:
                        # small sleep to avoid busy loop
                        time.sleep(0.01)

                if collected:
                    hexstr = collected.hex()
                    ints = ','.join(str(x) for x in collected)
                    try:
                        printable = collected.decode('utf-8', errors='replace')
                    except Exception:
                        printable = '<decode error>'
                    print(f"  Reply printable: {printable!r}  hex: {hexstr}  bytes: [{ints}]")
                else:
                    print('  No reply received')
                # Gap between repeats
                time.sleep(0.05)
    finally:
        try:
            ser.close()
        except Exception:
            pass

def main():
    print('RS485 sender: running single-byte tests (RTS toggle)')
    # single-byte bytes to test: 'C' and ';'
    send_single_byte_test([0x43, 0x3B], repeats=2, rts_settle=0.06, post_tx_wait=0.06)

    print('\nRS485 sender: trying RTS toggle method for multi-byte packet')
    # Try a blocked write: assert DE for the whole message and write bytes one-by-one
    send_blocked_message(MSG, repeats=1, rts_settle=0.06, inter_byte_delay=0.005, post_tx_wait=0.06)
    # Fallback: try single write approach as before
    send_with_rts()

    print('\nIf that did not work, trying pyserial RS485 helper (if installed)')
    send_with_rs485_helper()

    # --- Send a Modbus RTU read request using the blocked-message path ---
    print('\nNow sending a Modbus RTU Read Holding Registers (0x03) request')
    # Example: read 1 register starting at 0 from slave address 1
    try:
        reply = send_modbus_read(1, 0, 1, port=PORT, baud=BAUD, repeats=1, timeout=0.5, rts_settle=0.06, inter_byte_delay=0.005, post_tx_wait=0.08, de_control='RTS')
        if reply:
            print(f'Modbus reply (RTS): {reply.hex()}')
        else:
            print('No reply with RTS, retrying with DTR control...')
            reply = send_modbus_read(1, 0, 1, port=PORT, baud=BAUD, repeats=1, timeout=0.5, rts_settle=0.06, inter_byte_delay=0.005, post_tx_wait=0.08, de_control='DTR')
            if reply:
                print(f'Modbus reply (DTR): {reply.hex()}')
            else:
                print('No reply received (RTS and DTR attempts failed)')
    except Exception as e:
        print(f'Modbus send failed: {e}')

if __name__ == '__main__':
    main()
