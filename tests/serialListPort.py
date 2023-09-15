import serial.tools.list_ports
# ports = serial.tools.list_ports.comports()

# for port, desc, hwid in sorted(ports):
#         print(desc)
#         print("{}: {} [{}]".format(port, desc, hwid))


def getServoPort():
        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(ports):
                if desc == 'USB-SERIAL CH340 (COM10)':
                        return port
        print('Error: Servo is not connected to any port')
        return None

port = getServoPort()
print(port)