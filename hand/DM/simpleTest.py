import can
import time


can_interface = 'COM12'  # Replace with the actual port
bitrate = 1000000
canID = 0x01  # 设置 CAN ID
bus = can.interface.Bus(interface='slcan', channel=can_interface, bitrate=bitrate)

data=[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
print(bytearray(data))


# # 设置 ID 和数据
msg = can.Message(
    arbitration_id=canID,
    data=data,
    is_extended_id=False  # 标准帧
)

# # 发送消息
try:
    bus.send(msg)
    print(f"Message sent on {bus.channel_info}")
except can.CanError:
    print("Message NOT sent")

# Receive messages for 10 seconds
print("Listening for messages (10 seconds)...")
start_time = time.time()
while time.time() - start_time < 10:
    message = bus.recv(timeout=1.0)  # Wait up to 1 second
    if message is not None:
        print(f"Received message: {message}")

# Shutdown the bus properly
bus.shutdown()
print("Bus shutdown completed.")
