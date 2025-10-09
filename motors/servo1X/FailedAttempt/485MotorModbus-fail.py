"""
simple_modbus_rtu.py
Reliable Modbus RTU client using only pyserial - no external Modbus libraries.

Usage:
  python simple_modbus_rtu.py --port COM5 --slave 1 read_angle
  python simple_modbus_rtu.py --port COM5 --slave 1 enable_motor
  python simple_modbus_rtu.py --port COM5 --slave 1 set_angle 45.0
  python simple_modbus_rtu.py --port COM5 --slave 1 set_ratio 100
"""

import time
import serial
import struct
import argparse
from typing import Optional, List

# Register map (matches STM32 code)
REG_DEVICE_ID = 0
REG_ANGLE_RAW_HI = 1
REG_ANGLE_RAW_LO = 2
REG_ROTATION_COUNT_HI = 3
REG_ROTATION_COUNT_LO = 4
REG_TARGET_ANGLE_HI = 5
REG_TARGET_ANGLE_LO = 6
REG_REDUCTION_RATIO = 7
REG_MOTOR_ENABLE = 8

class SimpleModbusRTU:
    def __init__(self, port: str, baudrate: int = 115200, slave_id: int = 1, timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.ser = None
        
    def connect(self) -> bool:
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=self.timeout,
                rtscts=False,
                dsrdtr=False
            )
            # Important: Ensure RTS is controlled manually for RS485
            self.ser.rts = False
            print(f"✓ Connected to {self.port} at {self.baudrate} baud")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial port"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("✓ Disconnected")
    
    def calculate_crc(self, data: bytes) -> int:
        """Calculate Modbus CRC16"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc = crc >> 1
        return crc
    
    def send_modbus_frame(self, frame: bytes) -> Optional[bytes]:
        """Send Modbus frame and receive response"""
        if not self.ser or not self.ser.is_open:
            print("✗ Serial port not open")
            return None
        
        try:
            # Clear input buffer
            self.ser.reset_input_buffer()
            
            # Calculate CRC and build complete frame
            crc = self.calculate_crc(frame)
            full_frame = frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
            
            # Debug: print sent frame
            hex_frame = ' '.join(f'{b:02X}' for b in full_frame)
            print(f"→ Sending: {hex_frame}")
            
            # Send frame with RTS control for RS485
            self.ser.rts = True  # Enable transmitter
            time.sleep(0.001)    # Small delay for DE to activate
            self.ser.write(full_frame)
            self.ser.flush()
            time.sleep(0.005)    # Wait for transmission to complete
            self.ser.rts = False # Disable transmitter (back to receive)
            
            # Wait for response (minimum 5 bytes: addr + func + 2 data + 2 CRC)
            time.sleep(0.01)  # Short delay before reading
            response = self.ser.read(5)  # Read minimum expected bytes
            
            if len(response) < 5:
                print("✗ No response or incomplete frame")
                return None
            
            # Read remaining bytes based on function code
            if response[1] == 0x03:  # Read holding registers
                byte_count = response[2]
                remaining_bytes = byte_count + 2  # +2 for CRC
                remaining = self.ser.read(remaining_bytes)
                response += remaining
            elif response[1] in [0x06, 0x10]:  # Write single/multiple registers
                remaining_bytes = 4  # 4 bytes data + 2 CRC
                remaining = self.ser.read(remaining_bytes)
                response += remaining
            
            # Verify response CRC
            if len(response) >= 4:
                received_crc = (response[-1] << 8) | response[-2]
                calculated_crc = self.calculate_crc(response[:-2])
                
                if received_crc != calculated_crc:
                    print(f"✗ CRC error: received {received_crc:04X}, calculated {calculated_crc:04X}")
                    return None
            
            # Debug: print received frame
            hex_response = ' '.join(f'{b:02X}' for b in response)
            print(f"← Received: {hex_response}")
            
            return response
            
        except Exception as e:
            print(f"✗ Communication error: {e}")
            return None
    
    def read_holding_registers(self, address: int, count: int) -> Optional[List[int]]:
        """Read holding registers (Modbus function 0x03)"""
        frame = bytes([
            self.slave_id,          # Slave address
            0x03,                   # Function code
            (address >> 8) & 0xFF,  # Starting address high
            address & 0xFF,         # Starting address low
            (count >> 8) & 0xFF,    # Quantity high
            count & 0xFF           # Quantity low
        ])
        
        response = self.send_modbus_frame(frame)
        if not response or len(response) < 5:
            return None
        
        # Parse response: [addr, func, byte_count, data...]
        if response[0] != self.slave_id or response[1] != 0x03:
            return None
        
        byte_count = response[2]
        if byte_count != count * 2:
            return None
        
        registers = []
        for i in range(count):
            reg_data = response[3 + i*2:5 + i*2]
            if len(reg_data) == 2:
                registers.append((reg_data[0] << 8) | reg_data[1])
        
        return registers
    
    def write_single_register(self, address: int, value: int) -> bool:
        """Write single register (Modbus function 0x06)"""
        frame = bytes([
            self.slave_id,          # Slave address
            0x06,                   # Function code
            (address >> 8) & 0xFF,  # Address high
            address & 0xFF,         # Address low
            (value >> 8) & 0xFF,    # Value high
            value & 0xFF           # Value low
        ])
        
        response = self.send_modbus_frame(frame)
        if not response or len(response) < 6:
            return False
        
        # Response should echo the request
        return (response[0] == self.slave_id and 
                response[1] == 0x06 and
                response[2] == frame[2] and
                response[3] == frame[3] and
                response[4] == frame[4] and
                response[5] == frame[5])
    
    def read_angle(self) -> Optional[float]:
        """Read current angle as float"""
        registers = self.read_holding_registers(REG_ANGLE_RAW_HI, 2)
        if registers and len(registers) == 2:
            # Combine HI and LO registers to 32-bit integer
            angle_int = (registers[0] << 16) | registers[1]
            # Handle negative numbers (two's complement)
            if angle_int & 0x80000000:
                angle_int -= 0x100000000
            # Convert back to float (was scaled by 1000)
            return angle_int / 1000.0
        return None
    
    def read_rotation_count(self) -> Optional[int]:
        """Read rotation count as 32-bit integer"""
        registers = self.read_holding_registers(REG_ROTATION_COUNT_HI, 2)
        if registers and len(registers) == 2:
            rotation = (registers[0] << 16) | registers[1]
            # Handle negative rotation counts
            if rotation & 0x80000000:
                rotation -= 0x100000000
            return rotation
        return None
    
    def set_target_angle(self, angle: float) -> bool:
        """Set target angle"""
        # Scale float to 32-bit integer
        angle_int = int(angle * 1000)
        # Handle negative angles
        if angle_int < 0:
            angle_int += 0x100000000
        
        hi_word = (angle_int >> 16) & 0xFFFF
        lo_word = angle_int & 0xFFFF
        
        print(f"Setting target angle: {angle:.3f} rad -> HI:0x{hi_word:04X} LO:0x{lo_word:04X}")
        
        # Write both registers
        success1 = self.write_single_register(REG_TARGET_ANGLE_HI, hi_word)
        time.sleep(0.02)
        success2 = self.write_single_register(REG_TARGET_ANGLE_LO, lo_word)
        
        if success1 and success2:
            print("✓ Target angle set successfully")
            return True
        else:
            print("✗ Failed to set target angle")
            return False
    
    def set_reduction_ratio(self, ratio: int) -> bool:
        """Set reduction ratio"""
        if ratio < 1 or ratio > 1000:
            print(f"✗ Invalid ratio {ratio}, must be 1-1000")
            return False
        
        print(f"Setting reduction ratio: {ratio}")
        success = self.write_single_register(REG_REDUCTION_RATIO, ratio)
        if success:
            print("✓ Reduction ratio set successfully")
        else:
            print("✗ Failed to set reduction ratio")
        return success
    
    def enable_motor(self) -> bool:
        """Enable motor"""
        print("Enabling motor...")
        success = self.write_single_register(REG_MOTOR_ENABLE, 1)
        if success:
            print("✓ Motor enabled")
        else:
            print("✗ Failed to enable motor")
        return success
    
    def disable_motor(self) -> bool:
        """Disable motor"""
        print("Disabling motor...")
        success = self.write_single_register(REG_MOTOR_ENABLE, 0)
        if success:
            print("✓ Motor disabled")
        else:
            print("✗ Failed to disable motor")
        return success
    
    def read_all_registers(self) -> Optional[List[int]]:
        """Read all Modbus registers"""
        return self.read_holding_registers(0, 9)  # Read registers 0-8
    
    def read_device_info(self):
        """Read and display all device information"""
        registers = self.read_all_registers()
        if not registers or len(registers) < 9:
            print("✗ Failed to read device registers")
            return None
        
        print("\nDevice Information:")
        print(f"  Device ID: {registers[REG_DEVICE_ID]}")
        print(f"  Reduction Ratio: {registers[REG_REDUCTION_RATIO]}")
        print(f"  Motor Enabled: {registers[REG_MOTOR_ENABLE] != 0}")
        
        # Calculate angles
        angle_int = (registers[REG_ANGLE_RAW_HI] << 16) | registers[REG_ANGLE_RAW_LO]
        if angle_int & 0x80000000:
            angle_int -= 0x100000000
        current_angle = angle_int / 1000.0
        
        target_int = (registers[REG_TARGET_ANGLE_HI] << 16) | registers[REG_TARGET_ANGLE_LO]
        if target_int & 0x80000000:
            target_int -= 0x100000000
        target_angle = target_int / 1000.0
        
        rotation = (registers[REG_ROTATION_COUNT_HI] << 16) | registers[REG_ROTATION_COUNT_LO]
        if rotation & 0x80000000:
            rotation -= 0x100000000
        
        print(f"  Current Angle: {current_angle:.3f} rad")
        print(f"  Target Angle: {target_angle:.3f} rad")
        print(f"  Rotation Count: {rotation}")
        
        return {
            'device_id': registers[REG_DEVICE_ID],
            'current_angle': current_angle,
            'target_angle': target_angle,
            'rotation_count': rotation,
            'reduction_ratio': registers[REG_REDUCTION_RATIO],
            'motor_enabled': registers[REG_MOTOR_ENABLE] != 0
        }

def main():
    parser = argparse.ArgumentParser(description='Simple Modbus RTU Motor Controller Client')
    parser.add_argument('--port', default='COM5', help='Serial port (default: COM5)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--slave', type=int, default=1, help='Slave ID (default: 1)')
    parser.add_argument('--timeout', type=float, default=1.0, help='Timeout in seconds (default: 1.0)')
    
    # Commands
    parser.add_argument('command', choices=[
        'read_all', 'read_angle', 'enable_motor', 'disable_motor', 
        'set_angle', 'set_ratio', 'test', 'registers'
    ], help='Command to execute')
    
    parser.add_argument('value', nargs='?', help='Value for set commands (angle or ratio)')
    
    args = parser.parse_args()
    
    # Create Modbus client
    client = SimpleModbusRTU(args.port, args.baud, args.slave, args.timeout)
    
    if not client.connect():
        return
    
    try:
        if args.command == 'test':
            print("Testing communication...")
            registers = client.read_holding_registers(REG_DEVICE_ID, 1)
            if registers:
                print(f"✓ Communication successful - Device ID: {registers[0]}")
            else:
                print("✗ Communication failed")
                
        elif args.command == 'registers':
            print("Reading all registers...")
            registers = client.read_all_registers()
            if registers:
                for i, value in enumerate(registers):
                    print(f"Register {i}: 0x{value:04X} ({value})")
            else:
                print("✗ Failed to read registers")
                
        elif args.command == 'read_all':
            client.read_device_info()
                
        elif args.command == 'read_angle':
            angle = client.read_angle()
            if angle is not None:
                print(f"Current Angle: {angle:.3f} rad")
            else:
                print("✗ Failed to read angle")
                
        elif args.command == 'enable_motor':
            client.enable_motor()
                
        elif args.command == 'disable_motor':
            client.disable_motor()
                
        elif args.command == 'set_angle':
            if not args.value:
                print("✗ Error: Angle value required")
                return
            try:
                angle = float(args.value)
                client.set_target_angle(angle)
            except ValueError:
                print("✗ Error: Invalid angle value")
                
        elif args.command == 'set_ratio':
            if not args.value:
                print("✗ Error: Ratio value required")
                return
            try:
                ratio = int(args.value)
                client.set_reduction_ratio(ratio)
            except ValueError:
                print("✗ Error: Invalid ratio value")
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        client.disconnect()


def test():
    port = 'COM5'
    baud = 115200
    slave_id = 1
    timeout = 1.0

    client = SimpleModbusRTU(port, baud, slave_id, timeout)

    if not client.connect():
        print("✗ Cannot continue without connection")
        return

    registers = client.read_holding_registers(REG_DEVICE_ID, 1)
    if registers:
        print(f"✓ Communication successful - Device ID: {registers[0]}")

    client.set_target_angle(3.14)
    client.set_reduction_ratio(100)
    client.disable_motor()
    client.disable_motor()

    client.disconnect()

if __name__ == '__main__':
    test()