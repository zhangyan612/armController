#!/usr/bin/env python
import time
import serial
import sys
import platform
from protocol_packet_handler import *

BROADCAST_ID = 0xFE  # 254
MAX_ID = 0xFC  # 252
SCS_END = 0

# Instruction for SCS Protocol
INST_PING = 1
INST_READ = 2
INST_WRITE = 3
INST_REG_WRITE = 4
INST_ACTION = 5
INST_SYNC_WRITE = 131  # 0x83
INST_SYNC_READ = 130  # 0x82

# Communication Result
COMM_SUCCESS = 0  # tx or rx packet communication success
COMM_PORT_BUSY = -1  # Port is busy (in use)
COMM_TX_FAIL = -2  # Failed transmit instruction packet
COMM_RX_FAIL = -3  # Failed get status packet
COMM_TX_ERROR = -4  # Incorrect instruction packet
COMM_RX_WAITING = -5  # Now recieving status packet
COMM_RX_TIMEOUT = -6  # There is no status packet
COMM_RX_CORRUPT = -7  # Incorrect status packet
COMM_NOT_AVAILABLE = -9  #

DEFAULT_BAUDRATE = 1000000
LATENCY_TIMER = 50 

#波特率定义
SCSCL_1M = 0
SCSCL_0_5M = 1
SCSCL_250K = 2
SCSCL_128K = 3
SCSCL_115200 = 4
SCSCL_76800 = 5
SCSCL_57600 = 6
SCSCL_38400 = 7

#内存表定义
#-------EPROM(只读)--------
SCSCL_MODEL_L = 3
SCSCL_MODEL_H = 4

#-------EPROM(读写)--------
scs_id = 5
SCSCL_BAUD_RATE = 6
SCSCL_MIN_ANGLE_LIMIT_L = 9
SCSCL_MIN_ANGLE_LIMIT_H = 10
SCSCL_MAX_ANGLE_LIMIT_L = 11
SCSCL_MAX_ANGLE_LIMIT_H = 12
SCSCL_CW_DEAD = 26
SCSCL_CCW_DEAD = 27

#-------SRAM(读写)--------
SCSCL_TORQUE_ENABLE = 40
SCSCL_GOAL_POSITION_L = 42
SCSCL_GOAL_POSITION_H = 43
SCSCL_GOAL_TIME_L = 44
SCSCL_GOAL_TIME_H = 45
SCSCL_GOAL_SPEED_L = 46
SCSCL_GOAL_SPEED_H = 47
SCSCL_LOCK = 48

#-------SRAM(只读)--------
SCSCL_PRESENT_POSITION_L  = 56
SCSCL_PRESENT_POSITION_H = 57
SCSCL_PRESENT_SPEED_L = 58
SCSCL_PRESENT_SPEED_H = 59
SCSCL_PRESENT_LOAD_L = 60
SCSCL_PRESENT_LOAD_H = 61
SCSCL_PRESENT_VOLTAGE = 62
SCSCL_PRESENT_TEMPERATURE = 63
SCSCL_MOVING = 66
SCSCL_PRESENT_CURRENT_L = 69
SCSCL_PRESENT_CURRENT_H = 70

class GroupSyncWrite:
    def __init__(self, ph, start_address, data_length):
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.is_param_changed = False
        self.param = []
        self.data_dict = {}

        self.clearParam()

    def makeParam(self):
        if not self.data_dict:
            return

        self.param = []

        for scs_id in self.data_dict:
            if not self.data_dict[scs_id]:
                return

            self.param.append(scs_id)
            self.param.extend(self.data_dict[scs_id])

    def addParam(self, scs_id, data):
        if scs_id in self.data_dict:  # scs_id already exist
            return False

        if len(data) > self.data_length:  # input data is longer than set
            return False

        self.data_dict[scs_id] = data

        self.is_param_changed = True
        return True

    def removeParam(self, scs_id):
        if scs_id not in self.data_dict:  # NOT exist
            return

        del self.data_dict[scs_id]

        self.is_param_changed = True

    def changeParam(self, scs_id, data):
        if scs_id not in self.data_dict:  # NOT exist
            return False

        if len(data) > self.data_length:  # input data is longer than set
            return False

        self.data_dict[scs_id] = data

        self.is_param_changed = True
        return True

    def clearParam(self):
        self.data_dict.clear()

    def txPacket(self):
        if len(self.data_dict.keys()) == 0:
            return COMM_NOT_AVAILABLE

        if self.is_param_changed is True or not self.param:
            self.makeParam()

        return self.ph.syncWriteTxOnly(self.start_address, self.data_length, self.param,
                                       len(self.data_dict.keys()) * (1 + self.data_length))

class scscl(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 1)
        self.groupSyncWrite = GroupSyncWrite(self, SCSCL_GOAL_POSITION_L, 6)

    def WritePos(self, scs_id, position, time, speed):
        txpacket = [self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(time), self.scs_hibyte(time), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, SCSCL_GOAL_POSITION_L, len(txpacket), txpacket)

    def ReadPos(self, scs_id):
        scs_present_position, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SCSCL_PRESENT_POSITION_L)
        return scs_present_position, scs_comm_result, scs_error

    def ReadSpeed(self, scs_id):
        scs_present_speed, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, SCSCL_PRESENT_SPEED_L)
        return self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadPosSpeed(self, scs_id):
        scs_present_position_speed, scs_comm_result, scs_error = self.read4ByteTxRx(scs_id, SCSCL_PRESENT_POSITION_L)
        scs_present_position = self.scs_loword(scs_present_position_speed)
        scs_present_speed = self.scs_hiword(scs_present_position_speed)
        return scs_present_position, self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadMoving(self, scs_id):
        moving, scs_comm_result, scs_error = self.read1ByteTxRx(scs_id, SCSCL_MOVING)
        return moving, scs_comm_result, scs_error

    def SyncWritePos(self, scs_id, position, time, speed):
        txpacket = [self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(time), self.scs_hibyte(time), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.groupSyncWrite.addParam(scs_id, txpacket)

    def RegWritePos(self, scs_id, position, time, speed):
        txpacket = [self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(time), self.scs_hibyte(time), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.regWriteTxRx(scs_id, SCSCL_GOAL_POSITION_L, len(txpacket), txpacket)

    def RegAction(self):
        return self.action(BROADCAST_ID)

    def PWMMode(self, scs_id):
        txpacket = [0, 0, 0, 0]
        return self.writeTxRx(scs_id, SCSCL_MIN_ANGLE_LIMIT_L, len(txpacket), txpacket)

    def WritePWM(self, scs_id, time):
        return self.write2ByteTxRx(scs_id, SCSCL_GOAL_TIME_L, self.scs_toscs(time, 10))

    def LockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SCSCL_LOCK, 1)

    def unLockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, SCSCL_LOCK, 0)



class PortHandler(object):
    def __init__(self, port_name):
        self.is_open = False
        self.baudrate = DEFAULT_BAUDRATE
        self.packet_start_time = 0.0
        self.packet_timeout = 0.0
        self.tx_time_per_byte = 0.0

        self.is_using = False
        self.port_name = port_name
        self.ser = None

    def openPort(self):
        return self.setBaudRate(self.baudrate)

    def closePort(self):
        self.ser.close()
        self.is_open = False

    def clearPort(self):
        self.ser.flush()

    def setPortName(self, port_name):
        self.port_name = port_name

    def getPortName(self):
        return self.port_name

    def setBaudRate(self, baudrate):
        baud = self.getCFlagBaud(baudrate)

        if baud <= 0:
            # self.setupPort(38400)
            # self.baudrate = baudrate
            return False  # TODO: setCustomBaudrate(baudrate)
        else:
            self.baudrate = baudrate
            return self.setupPort(baud)

    def getBaudRate(self):
        return self.baudrate

    def getBytesAvailable(self):
        return self.ser.in_waiting

    def readPort(self, length):
        if (sys.version_info > (3, 0)):
            return self.ser.read(length)
        else:
            return [ord(ch) for ch in self.ser.read(length)]

    def writePort(self, packet):
        return self.ser.write(packet)

    def setPacketTimeout(self, packet_length):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = (self.tx_time_per_byte * packet_length) + (self.tx_time_per_byte * 3.0) + LATENCY_TIMER

    def setPacketTimeoutMillis(self, msec):
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = msec

    def isPacketTimeout(self):
        if self.getTimeSinceStart() > self.packet_timeout:
            self.packet_timeout = 0
            return True

        return False

    def getCurrentTime(self):
        return round(time.time() * 1000000000) / 1000000.0

    def getTimeSinceStart(self):
        time_since = self.getCurrentTime() - self.packet_start_time
        if time_since < 0.0:
            self.packet_start_time = self.getCurrentTime()

        return time_since

    def setupPort(self, cflag_baud):
        if self.is_open:
            self.closePort()

        self.ser = serial.Serial(
            port=self.port_name,
            baudrate=self.baudrate,
            # parity = serial.PARITY_ODD,
            # stopbits = serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS,
            timeout=0
        )

        self.is_open = True

        self.ser.reset_input_buffer()

        self.tx_time_per_byte = (1000.0 / self.baudrate) * 10.0

        return True

    def getCFlagBaud(self, baudrate):
        if baudrate in [4800, 9600, 14400, 19200, 38400, 57600, 115200, 128000, 250000, 500000, 1000000]:
            return baudrate
        else:
            return -1          