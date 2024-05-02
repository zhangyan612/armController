import time
import serial
import numpy as np
from protocol_packet_handler import *

# Constants and Definitions (from scservo_client.py)

# ... (Existing constants and definitions)

# SCServoClient Class

class SCServoClient:
    """Client for communicating with SCServo motors."""

    def __init__(self, motor_ids, port='/dev/ttyUSB0', baudrate=1000000, lazy_connect=False):
        """Initializes a new client.

        Args:
            motor_ids: All motor IDs being used by the client.
            port: The serial port to connect to.
            baudrate: The baudrate to use for communication.
            lazy_connect: If True, automatically connects when needed.
        """
        self.motor_ids = list(motor_ids)
        self.port_name = port
        self.baudrate = baudrate
        self.lazy_connect = lazy_connect

        self.port_handler = PortHandler(port)
        self.protocol_handler = scscl(self.port_handler)

        self._pos_reader = SCServoPosReader(self, self.motor_ids)
        self._vel_reader = SCServoVelReader(self, self.motor_ids)

    @property
    def is_connected(self):
        return self.port_handler.is_open

    def connect(self):
        """Connects to the SCServo motors."""
        assert not self.is_connected, 'Client is already connected.'
        if not self.port_handler.setBaudRate(self.baudrate):
            raise OSError(f"Failed to set baudrate to {self.baudrate}")
        self.port_handler.openPort()

    def disconnect(self):
        """Disconnects from the SCServo motors."""
        if self.is_connected:
            self.port_handler.closePort()

    def set_torque_enabled(self, motor_ids, enabled, retries=-1, retry_interval=0.25):
        """Sets whether torque is enabled for the motors."""
        for motor_id in motor_ids:
            while True:
                result, _, _ = self.protocol_handler.write1ByteTxRx(motor_id, SCSCL_TORQUE_ENABLE, 1 if enabled else 0)
                if result == COMM_SUCCESS:
                    break
                retries -= 1
                if retries == 0:
                    raise OSError(f"Failed to set torque for motor ID {motor_id}")
                time.sleep(retry_interval)

    def read_pos_vel_cur(self):
        """Returns the current positions, velocities, and currents (not supported)."""
        return self._pos_reader.read(), self._vel_reader.read(), None  # Currents not supported

    def read_pos(self):
        """Returns the current positions."""
        return self._pos_reader.read()

    def read_vel(self):
        """Returns the current velocities."""
        return self._vel_reader.read()

    def read_cur(self):
        """Current reading not supported for SCServo motors."""
        return None

    def write_desired_pos(self, motor_ids, positions):
        """Writes the given desired positions."""
        for motor_id, position in zip(motor_ids, positions):
            while True:
                result, _, _ = self.protocol_handler.WritePos(motor_id, int(position * 1024 / 300), 0, 0)
                if result == COMM_SUCCESS:
                    break
                raise OSError(f"Failed to write position for motor ID {motor_id}")

    # ... (Other helper methods as needed)

# SCServoReader Class (Base for Position and Velocity Readers)

class SCServoReader:
    """Base class for reading data from SCServo motors."""

    def __init__(self, client, motor_ids):
        self.client = client
        self.motor_ids = motor_ids

    def read(self):
        """Reads data from the motors (implemented in subclasses)."""
        raise NotImplementedError

# SCServoPosReader Class

class SCServoPosReader(SCServoReader):
    """Reads positions from SCServo motors."""

    def read(self):
        """Returns the current positions."""
        positions = []
        for motor_id in self.motor_ids:
            while True:
                position, result, _ = self.client.protocol_handler.ReadPos(motor_id)
                if result == COMM_SUCCESS:
                    positions.append(position * 300 / 1024)  # Convert to radians
                    break
                raise OSError(f"Failed to read position for motor ID {motor_id}")
        return np.array(positions)

# SCServoVelReader Class

class SCServoVelReader(SCServoReader):
    """Reads velocities from SCServo motors."""

    def read(self):
        """Returns the current velocities."""
        velocities = []
        for motor_id in self.motor_ids:
            while True:
                velocity, result, _ = self.client.protocol_handler.ReadSpeed(motor_id)
                if result == COMM_SUCCESS:
                    velocities.append(velocity * 300 / 1024 * 0.111) 
                    break
                raise OSError(f"Failed to read velocity for motor ID {motor_id}") 
        return np.array(velocities)
