import time
import json
from pynput import mouse, keyboard
import math
import paho.mqtt.client as mqtt
import os

# MQTT Configuration
TOPIC = "remote_keyboard_mouse"
MQTT_CONFIG_PATH = "mqtt_config.json"

# Position cache and thresholds
current_pos = {'x': 0, 'y': 0}
last_sent_pos = {'x': 0, 'y': 0}
MOVE_THRESHOLD = 10

def distance(p1, p2):
    return math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])

def load_config():
    with open(MQTT_CONFIG_PATH, "r") as f:
        return json.load(f)

def setup_mqtt(cfg):
    client = mqtt.Client()
    client.connect(cfg["broker"], cfg["port"], 60)
    client.loop_start()
    return client

# Mouse event handlers
def on_move(x, y):
    global current_pos, last_sent_pos
    current_pos = {'x': int(x), 'y': int(y)}
    
    if distance(current_pos, last_sent_pos) >= MOVE_THRESHOLD:
        payload = {
            'type': 'mouse_move',
            'x': current_pos['x'],
            'y': current_pos['y']
        }
        mqtt_client.publish(TOPIC, json.dumps(payload))
        last_sent_pos = current_pos.copy()

def on_click(x, y, button, pressed):
    payload = {
        'type': 'mouse_click',
        'x': int(x),
        'y': int(y),
        'button': button.name,
        'action': 'down' if pressed else 'up'
    }
    mqtt_client.publish(TOPIC, json.dumps(payload))

def on_scroll(x, y, dx, dy):
    payload = {
        'type': 'mouse_scroll',
        'x': int(x),
        'y': int(y),
        'dx': int(dx),
        'dy': int(dy)
    }
    mqtt_client.publish(TOPIC, json.dumps(payload))

# Keyboard event handlers
def on_press(key):
    try:
        k = key.char
        payload = {
            'type': 'key_press',
            'key': k,
            'action': 'press'
        }
    except AttributeError:
        k = key.name
        payload = {
            'type': 'key_press',
            'key': k,
            'action': 'press'
        }
    
    mqtt_client.publish(TOPIC, json.dumps(payload))

def on_release(key):
    try:
        k = key.char
    except AttributeError:
        k = key.name
        
    payload = {
        'type': 'key_release',
        'key': k,
        'action': 'release'
    }
    mqtt_client.publish(TOPIC, json.dumps(payload))

if __name__ == "__main__":
    # MQTT Setup
    cfg = load_config()
    mqtt_client = setup_mqtt(cfg)
    
    # Start listeners
    mouse_listener = mouse.Listener(
        on_move=on_move, 
        on_click=on_click,
        on_scroll=on_scroll
    )
    keyboard_listener = keyboard.Listener(
        on_press=on_press, 
        on_release=on_release
    )
    
    mouse_listener.start()
    keyboard_listener.start()
    
    print("Input publisher started. Capturing events...")
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping publisher...")
        mouse_listener.stop()
        keyboard_listener.stop()
        mqtt_client.loop_stop()