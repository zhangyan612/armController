from servoControl import FeetechServoController, clickServo

import serial.tools.list_ports

def find_serial_ports(target_name=None):
    ports = list(serial.tools.list_ports.comports())
    found_ports = []

    for port in ports:
        port_info = {
            'device': port.device,
            'name': port.name,
            'description': port.description,
            'hwid': port.hwid,
            'manufacturer': port.manufacturer,
            'product': port.product,
            'serial_number': port.serial_number
        }

        if target_name and target_name.lower() in (port.description or '').lower():
            print(f"🟢 Found matching device: {port.device} - {port.description}")
            found_ports.append(port.device)
        else:
            print(f"🔹 {port.device} - {port.description}")

    return found_ports

# Example: Find all ports, highlight those with 'CH340'
matching_ports = find_serial_ports("CH340")

print("\nMatching Ports:", matching_ports)

# if len(matching_ports)  > 0:
controller = FeetechServoController(port='COM14', baudrate=1000000)
clickServo(controller, 2) # laptop 

