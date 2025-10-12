import time
import smbus2
import os

class RDKX3Encoder:
    def __init__(self, i2c_bus=0, retry_count=2, retry_delay=1.0):
        """
        Initialize RDK-X3 encoder reader
        
        Args:
            i2c_bus (int): I2C bus number (default: 0 for RDK-X3 based on your scan)
            retry_count (int): Number of retries for failed reads
            retry_delay (float): Delay between retries in seconds
        """
        # TCA9548A addresses
        self.TCA9548A_ADDRESSES = [0x70, 0x71, 0x72]
        
        # MT6701 parameters
        self.MT6701_ADDRESS = 0x06
        self.ANGLE_REGISTER = 0x03
        self.COUNTS_PER_REV = 16384
        self.COUNTS_TO_DEGREES = 360.0 / self.COUNTS_PER_REV
        
        self.i2c_bus = i2c_bus
        self.retry_count = retry_count
        self.retry_delay = retry_delay
        
        # Check I2C device existence and permissions
        i2c_device = f"/dev/i2c-{i2c_bus}"
        if not os.path.exists(i2c_device):
            raise FileNotFoundError(f"I2C device {i2c_device} does not exist")
        
        # Check if we have read/write permissions
        if not os.access(i2c_device, os.R_OK | os.W_OK):
            print(f"Warning: No read/write permissions for {i2c_device}")
            print("Try running with sudo or add user to i2c group:")
            print("sudo usermod -a -G i2c $USER")
            print("Then logout and login again")
        
        # Initialize I2C bus
        try:
            self.bus = smbus2.SMBus(i2c_bus)
            print(f"RDK-X3 Encoder initialized on I2C bus {i2c_bus}")
            
            # Verify we can communicate with TCA devices
            self.verify_tca_devices()
            
        except Exception as e:
            print(f"Error initializing I2C bus {i2c_bus}: {e}")
            raise
    
    def verify_tca_devices(self):
        """Verify TCA9548 devices are accessible"""
        print("Verifying TCA9548 devices...")
        for addr in self.TCA9548A_ADDRESSES:
            try:
                # Try to write to the device (disable all channels)
                self.bus.write_byte(addr, 0x00)
                print(f"✓ TCA9548A at {hex(addr)} is accessible")
            except Exception as e:
                print(f"✗ TCA9548A at {hex(addr)} failed: {e}")
    
    def disable_all_tca(self):
        """Disable all TCA9548A multiplexer channels"""
        success = True
        for tca_addr in self.TCA9548A_ADDRESSES:
            try:
                self.bus.write_byte(tca_addr, 0x00)
            except Exception as e:
                print(f"Error disabling TCA at {hex(tca_addr)}: {e}")
                success = False
        return success
    
    def select_mux_channel(self, tca_addr, channel):
        """Select specific channel on TCA9548A multiplexer"""
        if channel > 7:
            return False
        
        if not self.disable_all_tca():
            return False
            
        try:
            self.bus.write_byte(tca_addr, 1 << channel)
            time.sleep(0.0003)  # 300 microseconds delay
            return True
        except Exception as e:
            print(f"Error selecting channel {channel} on TCA {hex(tca_addr)}: {e}")
            return False
    
    def read_mt6701_angle_with_retry(self):
        """Read angle from MT6701 sensor with retry logic"""
        last_error = None
        
        for attempt in range(self.retry_count + 1):
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
                    
                    if attempt > 0:
                        print(f"  ✓ Success after {attempt + 1} attempts")
                    return angle_deg
                else:
                    last_error = "Incomplete data received"
                    
            except Exception as e:
                last_error = str(e)
                if attempt < self.retry_count:
                    print(f"  Retry {attempt + 1}/{self.retry_count} after error: {e}")
                    time.sleep(self.retry_delay)
                else:
                    print(f"  Failed after {self.retry_count + 1} attempts: {e}")
        
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
            return self.read_mt6701_angle_with_retry()
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
            print(f"Reading Board {board_id}, Port {port}...")
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

# Test with improved error handling
if __name__ == "__main__":
    try:
        # Initialize with bus 0 and retry settings
        encoder = RDKX3Encoder(i2c_bus=0, retry_count=2, retry_delay=1.0)
        
        # Define your query list
        query_list = [
            (70, 3),  # Left shoulder
            (70, 5),  # Right shoulder
            (71, 1),  # Left arm wrist 1
            (71, 7),  # Left arm wrist 2
            (71, 8),  # Left arm elbow
            (72, 4),  # Right arm elbow
            (72, 6),  # Right arm wrist 1
            (72, 7),  # Right arm wrist 2
        ]
        
        print("\nReading encoder values with retry...")
        result = encoder.query_batch(query_list)
        
        print("\nFinal Encoder Readings:")
        successful_reads = 0
        for reading in result:
            board = reading["board"]
            port = reading["port"]
            angle = reading["angle"]
            if angle is not None:
                status = f"{angle}°"
                successful_reads += 1
            else:
                status = "ERROR"
            print(f"  Board {board}, Port {port}: {status}")
        
        print(f"\nSuccessfully read {successful_reads}/{len(query_list)} encoders")
            
    except Exception as e:
        print(f"Failed to initialize encoder: {e}")
    finally:
        if 'encoder' in locals():
            encoder.close()