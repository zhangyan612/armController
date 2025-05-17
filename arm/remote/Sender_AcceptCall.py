import time
import json
import paho.mqtt.client as mqtt
from pynput.mouse import Controller, Button

MQTT_BROKER = "broker.mqttdashboard.com"
MQTT_PORT = 1883
TOPIC = "yzhang/mouse_keyboard"

client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)

mouse = Controller()

def send_event(event_type, data):
    payload = {'type': event_type, **data}
    client.publish(TOPIC, json.dumps(payload))

def move_mouse(x, y):
    mouse.position = (x, y)
    send_event("move", {"x": x, "y": y})
    print(f"Mouse moved to: ({x}, {y})")

def left_click():
    mouse.click(Button.left, 1)
    send_event("mouse", {"x": mouse.position[0], "y": mouse.position[1], "button": "left", "action": "down"})
    print("Left click performed")

    time.sleep(0.1)  # Small delay for natural click feel

    mouse.release(Button.left)
    send_event("mouse", {"x": mouse.position[0], "y": mouse.position[1], "button": "left", "action": "up"})
    print("Left button released")


# accept call
target_x, target_y = 1582, 955  # Define the target position
move_mouse(target_x, target_y)

time.sleep(1)
left_click()

time.sleep(5)


target_x, target_y = 821, 883  # Define the target position
move_mouse(target_x, target_y)

time.sleep(1)
left_click()

time.sleep(5)


target_x, target_y = 485, 419  # Define the target position
move_mouse(target_x, target_y)

time.sleep(1)
left_click()

time.sleep(10)

target_x, target_y = 998, 541  # Define the target position
move_mouse(target_x, target_y)

time.sleep(1)
left_click()


# Keep MQTT alive
while True:
    client.loop()
    time.sleep(2)



# Example usage
# target_x, target_y = 1100, 460  # Define the target position
# move_mouse(target_x, target_y)
# time.sleep(2)
# left_click()


# target_x, target_y = 1600, 954  # Define the target position
# move_mouse(target_x, target_y)
# time.sleep(2)
# left_click()

# # Keep MQTT alive
# while True:
#     client.loop()
#     time.sleep(2)