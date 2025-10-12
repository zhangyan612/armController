import time
import smbus2

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
        
        # Initialize I2C bus
        try:
            self.bus = smbus2.SMBus(i2c_bus)
            print(f"RDK-X3 Encoder initialized on I2C bus {i2c_bus}")
        except Exception as e:
            print(f"Error initializing I2C bus: {e}")
            raise
    
    def disable_all_tca(self):
        """Disable all TCA9548A multiplexer channels"""
        for tca_addr in self.TCA9548A_ADDRESSES:
            try:
                self.bus.write_byte(tca_addr, 0x00)
            except Exception as e:
                print(f"Error disabling TCA at {hex(tca_addr)}: {e}")
    
    def select_mux_channel(self, tca_addr, channel):
        """Select specific channel on TCA9548A multiplexer"""
        if channel > 7:
            return False
        
        self.disable_all_tca()
        try:
            self.bus.write_byte(tca_addr, 1 << channel)
            time.sleep(0.0003)  # 300 microseconds delay
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

# Usage example
if __name__ == "__main__":
    # Initialize the encoder reader
    encoder = RDKX3Encoder(i2c_bus=1)
    
    try:
        # Define the query list
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
        
        angle = encoder.query_single(70, 3)
        print(f"Single Query - Board 70, Port 3: {angle}°")

        # Query all encoders
        result = encoder.query_batch(query_list)
        
        # Print results
        print("Encoder Readings:")
        for reading in result:
            board = reading["board"]
            port = reading["port"]
            angle = reading["angle"]
            status = f"{angle}°" if angle is not None else "Error"
            print(f"  Board {board}, Port {port}: {status}")
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        encoder.close()