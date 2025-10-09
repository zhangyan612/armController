import serial
import time
import struct

def modbus_crc(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc = crc >> 1
    return crc

def test_read_device_id(port='COM5'):
    """Test reading device ID (register 0)"""
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        ser.rts = False
        
        # Read device ID - function 0x03, register 0, count 1
        frame = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
        crc = modbus_crc(frame)
        full_frame = frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        
        print(f"→ Sending: {' '.join(f'{b:02X}' for b in full_frame)}")
        
        # Clear buffer
        ser.reset_input_buffer()
        
        # Send with RTS - improved timing
        ser.rts = True
        time.sleep(0.002)  # 2ms delay for DE to activate
        ser.write(full_frame)
        ser.flush()
        time.sleep(0.002)  # 2ms delay before disabling DE
        ser.rts = False
        
        # Read response with longer timeout
        time.sleep(0.01)  # 10ms delay before reading
        response = ser.read(256)  # Read more bytes
        
        if response:
            hex_response = ' '.join(f'{b:02X}' for b in response)
            print(f"← Received: {hex_response} (len: {len(response)})")
            
            if len(response) >= 7:
                # Verify CRC
                recv_crc = (response[-1] << 8) | response[-2]
                calc_crc = modbus_crc(response[:-2])
                
                if recv_crc == calc_crc:
                    device_id = (response[3] << 8) | response[4]  # Data in bytes 3-4
                    print(f"✓ Device ID: {device_id}")
                    return True
                else:
                    print(f"✗ CRC error: calc={calc_crc:04X}, recv={recv_crc:04X}")
            else:
                print("✗ Incomplete response")
        else:
            print("✗ No response - Check wiring and STM32 configuration")
            
        return False
        
    except Exception as e:
        print(f"Error: {e}")
        return False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

def test_write_register(port='COM5'):
    """Test writing to a register"""
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        ser.rts = False
        
        # Write to register 7 with value 100
        frame = bytes([0x01, 0x06, 0x00, 0x07, 0x00, 0x64])
        crc = modbus_crc(frame)
        full_frame = frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        
        print(f"→ Sending: {' '.join(f'{b:02X}' for b in full_frame)}")
        
        # Clear buffer
        ser.reset_input_buffer()
        
        # Send with RTS
        ser.rts = True
        time.sleep(0.002)
        ser.write(full_frame)
        ser.flush()
        time.sleep(0.002)
        ser.rts = False
        
        # Read response
        time.sleep(0.01)
        response = ser.read(256)
        
        if response:
            hex_response = ' '.join(f'{b:02X}' for b in response)
            print(f"← Received: {hex_response} (len: {len(response)})")
            
            if len(response) >= 8:
                # Should echo the same frame
                if response[:6] == frame:
                    print("✓ Write successful")
                    return True
                else:
                    print("✗ Write failed - response doesn't match")
            else:
                print("✗ Incomplete response")
        else:
            print("✗ No response")
            
        return False
        
    except Exception as e:
        print(f"Error: {e}")
        return False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

# Add a simple echo test first
def test_echo(port='COM5'):
    """Simple test to verify basic communication"""
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        ser.rts = False
        
        # Send a simple pattern
        test_data = bytes([0x01, 0x02, 0x03, 0x04])
        print(f"→ Echo test sending: {' '.join(f'{b:02X}' for b in test_data)}")
        
        ser.reset_input_buffer()
        
        ser.rts = True
        time.sleep(0.002)
        ser.write(test_data)
        ser.flush()
        time.sleep(0.002)
        ser.rts = False
        
        time.sleep(0.01)
        response = ser.read(256)
        
        if response:
            print(f"← Echo received: {' '.join(f'{b:02X}' for b in response)}")
            return True
        else:
            print("✗ No echo response")
            return False
            
    except Exception as e:
        print(f"Echo test error: {e}")
        return False
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    print("Testing Modbus communication with STM32...")
    
    print("\n0. Testing basic echo:")
    if test_echo():
        print("✓ Echo test passed")
    else:
        print("✗ Echo test failed")
    
    print("\n1. Testing read device ID:")
    if test_read_device_id():
        print("✓ Read test passed")
    else:
        print("✗ Read test failed")
    
    print("\n2. Testing write register:")
    if test_write_register():
        print("✓ Write test passed")
    else:
        print("✗ Write test failed")