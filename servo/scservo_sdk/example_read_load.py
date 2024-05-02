#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(SCS), and an URT
#

import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
        
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from scservo_sdk import *                      # Uses SCServo SDK library

# Default setting
SCS_ID                      = 1                 # SCServo ID : 1
BAUDRATE                    = 115200           # SCServo default baudrate : 1000000
DEVICENAME                  = 'COM14'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = scscl(portHandler)
    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break

    scs_present_load, scs_comm_result, scs_error = packetHandler.ReadLoad(SCS_ID)
    if scs_comm_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(scs_comm_result))
    else:
        print("[ID:%03d] Present Load:%d" % (SCS_ID, scs_present_load))
        print(scs_comm_result)

    if scs_error != 0:
        print(packetHandler.getRxPacketError(scs_error))

# Close port
portHandler.closePort()
