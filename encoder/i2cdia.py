import time
import smbus2
import subprocess

class RDKX3Encoder:
    def __init__(self, i2c_bus=1):
        """
        Initialize RDK-X3 encoder reader
        
        Args:
            i2c_bus (int): I2C bus number (default: 1 for RDK-X3)
        """
        # TCA9548A addresses
        self.TCA9548A_ADDRESSES = [0x70, 0x71, 0x72]
        
        # MT6701 parameters
        self.MT6701_ADDRESS = 0x06
        self.ANGLE_REGISTER = 0x03
        self.COUNTS_PER_REV = 16384
        self.COUNTS_TO_DEGREES = 360.0 / self.COUNTS_PER_REV
        
        self.i2c_bus = i2c_bus
        
        # Initialize I2C bus
        try:
            self.bus = smbus2.SMBus(i2c_bus)
            print(f"RDK-X3 Encoder initialized on I2C bus {i2c_bus}")
            
            # Scan for available devices
            self.scan_i2c_devices()
            
        except Exception as e:
            print(f"Error initializing I2C bus {i2c_bus}: {e}")
            print("Available I2C buses on RDK-X3:")
            self.list_i2c_buses()
            raise
    
    def list_i2c_buses(self):
        """List available I2C buses on the system"""
        try:
            # Try to find I2C devices
            result = subprocess.run(['i2cdetect', '-l'], capture_output=True, text=True)
            print("I2C buses available:")
            print(result.stdout)
        except:
            print("i2cdetect not available, trying alternative method...")
            try:
                # Alternative method to check I2C buses
                import os
                if os.path.exists('/dev/i2c-0'):
                    print("/dev/i2c-0 exists")
                if os.path.exists('/dev/i2c-1'):
                    print("/dev/i2c-1 exists")
                if os.path.exists('/dev/i2c-2'):
                    print("/dev/i2c-2 exists")
            except:
                print("Could not determine I2C buses")
    
    def scan_i2c_devices(self):
        """Scan for I2C devices on the bus"""
        print(f"Scanning I2C bus {self.i2c_bus} for devices...")
        found_devices = []
        
        for address in range(0x03, 0x78):
            try:
                self.bus.read_byte(address)
                found_devices.append(hex(address))
                print(f"Found device at address: {hex(address)}")
            except:
                pass
        
        if found_devices:
            print(f"Found {len(found_devices)} I2C device(s): {found_devices}")
        else:
            print("No I2C devices found!")
        
        # Check specifically for our expected devices
        for addr in self.TCA9548A_ADDRESSES:
            try:
                self.bus.read_byte(addr)
                print(f"✓ TCA9548A found at {hex(addr)}")
            except:
                print(f"✗ TCA9548A NOT found at {hex(addr)}")
        
        try:
            self.bus.read_byte(self.MT6701_ADDRESS)
            print(f"✓ MT6701 found at {hex(self.MT6701_ADDRESS)}")
        except:
            print(f"✗ MT6701 NOT found at {hex(self.MT6701_ADDRESS)}")
    
    def disable_all_tca(self):
        """Disable all TCA9548A multiplexer channels"""
        errors = []
        for tca_addr in self.TCA9548A_ADDRESSES:
            try:
                self.bus.write_byte(tca_addr, 0x00)
                #print(f"Disabled TCA at {hex(tca_addr)}")
            except Exception as e:
                errors.append(f"{hex(tca_addr)}: {e}")
        
        if errors:
            print(f"Errors disabling TCA devices: {errors}")
            return False
        return True
    
    def select_mux_channel(self, tca_addr, channel):
        """Select specific channel on TCA9548A multiplexer"""
        if channel > 7:
            return False
        
        # First verify the TCA device exists
        try:
            self.bus.read_byte(tca_addr)
        except:
            print(f"TCA device not responding at {hex(tca_addr)}")
            return False
        
        if not self.disable_all_tca():
            return False
            
        try:
            self.bus.write_byte(tca_addr, 1 << channel)
            time.sleep(0.0003)  # 300 microseconds delay
            #print(f"Selected channel {channel} on TCA {hex(tca_addr)}")
            return True
        except Exception as e:
            print(f"Error selecting channel {channel} on TCA {hex(tca_addr)}: {e}")
            return False
    
    def read_mt6701_angle(self):
        """Read angle from MT6701 sensor"""
        try:
            # Write the register address we want to read from
            self.bus.write_byte(self.MT6701_ADDRESS, self.ANGLE_REGISTER)
            time.sleep(0.001)  # Short delay
            
            # Read 2 bytes from the device
            data = self.bus.read_i2c_block_data(self.MT6701_ADDRESS, self.ANGLE_REGISTER, 2)
            
            if len(data) == 2:
                angle_h = data[0]
                angle_l = data[1]
                angle_count = (angle_h << 6) | (angle_l & 0x3F)
                angle_deg = angle_count * self.COUNTS_TO_DEGREES
                return angle_deg
            else:
                return None
        except Exception as e:
            print(f"Error reading MT6701: {e}")
            return None
    
    def get_board_address(self, board_id):
        """Convert board ID to TCA9548A address"""
        if board_id == 70:
            return 0x70
        elif board_id == 71:
            return 0x71
        elif board_id == 72:
            return 0x72
        else:
            return None
    
    def query_single(self, board_id, port):
        """
        Query a single encoder
        
        Args:
            board_id (int): TCA9548A board ID (70, 71, or 72)
            port (int): Port number (1-8)
        
        Returns:
            float or None: Angle in degrees, or None if error
        """
        tca_addr = self.get_board_address(board_id)
        if tca_addr is None:
            print(f"Invalid board ID: {board_id}")
            return None
        
        if port < 1 or port > 8:
            print(f"Invalid port: {port}. Must be between 1-8")
            return None
        
        # Select multiplexer channel (port-1 because ports are 1-based)
        channel = port - 1
        if self.select_mux_channel(tca_addr, channel):
            time.sleep(0.005)  # 5ms delay for sensor stabilization
            return self.read_mt6701_angle()
        else:
            return None
    
    def query_batch(self, query_list):
        """
        Query multiple encoders in batch
        
        Args:
            query_list (list): List of tuples [(board_id, port), ...]
            
        Returns:
            list: List of dictionaries with results
        """
        results = []
        
        for board_id, port in query_list:
            angle = self.query_single(board_id, port)
            
            results.append({
                "board": board_id,
                "port": port,
                "angle": round(angle, 2) if angle is not None else None
            })
        
        return results
    
    def close(self):
        """Clean up and close I2C connection"""
        self.disable_all_tca()
        print("RDK-X3 Encoder closed")

# Test function
def test_encoder_reading():
    """Test function to diagnose the encoder reading issues"""
    
    # Try different I2C buses
    buses_to_try = [1, 0, 2]
    
    for bus_num in buses_to_try:
        print(f"\n{'='*50}")
        print(f"Trying I2C bus {bus_num}")
        print(f"{'='*50}")
        
        try:
            encoder = RDKX3Encoder(i2c_bus=bus_num)
            
            # Test query
            query_list = [
                (70, 3),  # Left shoulder
                (70, 5),  # Right shoulder
            ]
            
            print("\nTesting encoder readings...")
            result = encoder.query_batch(query_list)
            
            print("Results:")
            for reading in result:
                board = reading["board"]
                port = reading["port"]
                angle = reading["angle"]
                status = f"{angle}°" if angle is not None else "Error"
                print(f"  Board {board}, Port {port}: {status}")
            
            encoder.close()
            break  # Stop if successful
            
        except Exception as e:
            print(f"Failed on bus {bus_num}: {e}")
            continue

# Usage example
if __name__ == "__main__":
    # First run the diagnostic test
    test_encoder_reading()