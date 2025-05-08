import time
import json
import paho.mqtt.client as mqtt
from pynput import mouse, keyboard

# pip install paho-mqtt

MQTT_BROKER = "broker.mqttdashboard.com"
MQTT_PORT = 1883
TOPIC = "yzhang/mouse_keyboard"

client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Position cache
current_pos = {'x': 0, 'y': 0}

def send_event(event_type, data):
    payload = {'type': event_type, **data}
    client.publish(TOPIC, json.dumps(payload))

# Mouse callbacks
def on_move(x, y):
    global current_pos
    current_pos = {'x': int(x), 'y': int(y)}
    send_event('move', current_pos)

def on_click(x, y, button, pressed):
    send_event('mouse', {
        'x': int(x),
        'y': int(y),
        'button': button.name,
        'action': 'down' if pressed else 'up'
    })

# Keyboard callbacks
def on_press(key):
    try:
        k = key.char
    except AttributeError:
        k = key.name
    send_event('keyboard', {'key': k, 'action': 'press'})

def on_release(key):
    try:
        k = key.char
    except AttributeError:
        k = key.name
    send_event('keyboard', {'key': k, 'action': 'release'})

# Start listeners
mouse.Listener(on_move=on_move, on_click=on_click).start()
keyboard.Listener(on_press=on_press, on_release=on_release).start()

# Keep MQTT alive
while True:
    client.loop()
    time.sleep(0.01)
