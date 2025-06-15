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

def video_call_accept():
    # china desktop call control
    # accept call
    target_x, target_y = 1862, 971  # Define the target position
    move_mouse(target_x, target_y)

    time.sleep(1)
    left_click()


def accept_control():
    # accept control
    target_x, target_y = 1091, 564  # Define the target position
    move_mouse(target_x, target_y)

    time.sleep(1)
    left_click()


def share_screen():
    # share screen
    target_x, target_y = 912, 904  # Define the target position
    move_mouse(target_x, target_y)  

    time.sleep(1)
    left_click()

    time.sleep(5)

    # choose screen
    target_x, target_y = 615, 443  # Define the target position
    move_mouse(target_x, target_y)

    time.sleep(1)
    left_click()


def full_accept():
    video_call_accept()
    time.sleep(5)
    share_screen()

# Keep MQTT alive
while True:
    client.loop()

    user_input = input("Type 'v' to accept call, c for control, f for all control: ")
    if user_input.strip().lower() == "v":
        video_call_accept()
    elif user_input.strip().lower() == "s":
        share_screen()
    elif user_input.strip().lower() == "c":
        accept_control()
    elif user_input.strip().lower() == "f":
        full_accept()
    else:
        print("Waiting for 'start'...")

    time.sleep(1)



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