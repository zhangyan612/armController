# import can

# # READ SUCCESS
# # Configure the CAN bus using the SLCAN interface
# bus = can.interface.Bus(interface='slcan', channel='COM12', bitrate=500000)

# try:
#     while True:
#         # Read a message from the CAN bus
#         msg = bus.recv()
#         if msg:
#             print(f"{msg.arbitration_id:X}:  {msg.data}")
# except KeyboardInterrupt:
#     print("Stopped by user")
# finally:
#     bus.shutdown()


import can

# Configure the CAN bus using the SLCAN interface
bus = can.interface.Bus(interface='slcan', channel='COM12', bitrate=500000)

# Create a CAN message to send
send_msg = can.Message(arbitration_id=0x1E, data=[0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08], is_extended_id=False)

try:
    # Send the CAN message
    bus.send(send_msg)
    print("Message sent on CAN bus")
    
    # Read only the first response 
    response_msg = bus.recv()
    if response_msg: 
        print(f"Received response - {response_msg.arbitration_id:X}: {response_msg.data}")
    # Loop to receive responses
    # while True:
    #     # Read a message from the CAN bus
    #     response_msg = bus.recv()
    #     if response_msg:
    #         print(f"Received response - {response_msg.arbitration_id:X}: {response_msg.data}")
    #     else:
    #         break

except can.CanError:
    print("Message NOT sent")

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    bus.shutdown()
