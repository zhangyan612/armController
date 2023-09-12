import serial
import time

# # Replace 'COM3' with the appropriate port name for your system
# serial_port = 'COM5'
# baud_rate = 2250000

# # Establish a connection to the serial port
# ser = serial.Serial(serial_port, baud_rate, timeout=1)

# try:
#     while True:
#         # Read data from the serial port
#         data = ser.readline().decode('utf-8').strip()

#         # If data received, print it
#         if data:
#             print("Received data from serial port: ", data)
#             # Give the device time to send data again
#             time.sleep(0.5)

# # To close the serial port gracefully, use Ctrl+C to break the loop
# except KeyboardInterrupt:
#     print("Closing the serial port.")
#     ser.close()



with serial.Serial('COM5', 2250000, timeout=1) as ser:
     x = ser.read()          # read one byte
     s = ser.read(10)        # read up to ten bytes (timeout)
     line = ser.readline()   # read a '\n' terminated line
     print(line)
