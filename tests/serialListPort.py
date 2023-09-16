import serial.tools.list_ports
# ports = serial.tools.list_ports.comports()

# for port, desc, hwid in sorted(ports):
#         print(desc)
#         print("{}: {} [{}]".format(port, desc, hwid))


def getServoPort():
        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(ports):
                print(port)
                print(desc)
                if 'USB-SERIAL CH340' in desc:
                        return port
        print('Error: Servo is not connected to any port')
        return None

port = getServoPort()
print(port)