import can
print(can.detect_available_configs())



# [{'interface': 'serial', 'channel': 'COM9'}, {'interface': 'serial', 'channel': 'COM10'}, {'interface': 'serial', 'channel': 'COM5'}, {'interface': 'virtual', 'channel': 'channel-4964'}]



# import can

# bus = can.interface.Bus(
#     bustype='slcan',
#     channel='COM5',   # Replace with your actual COM port
#     bitrate=500000
# )

# msg = can.Message(arbitration_id=0x123, data=[1,2,3], is_extended_id=False)
# bus.send(msg)
# print("Message sent.")
