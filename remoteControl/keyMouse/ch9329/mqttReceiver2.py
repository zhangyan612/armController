import json
import paho.mqtt.client as mqtt
from pynput import mouse, keyboard
import os
import paho

def load_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    config_path = os.path.join(root_dir, "mqtt_config.json")
    with open(config_path, "r") as f:
        return json.load(f)

# cfg = load_config()
# MQTT Configuration
# MQTT_BROKER = cfg["broker"]
# MQTT_PORT = 1883

TOPIC = "remote_keyboard_mouse"

# Initialize MQTT client
# client = mqtt.Client()
# client.connect(MQTT_BROKER, MQTT_PORT, 60)
# client.loop_start()


cfg = load_config()
client = mqtt.Client()
# client.on_connect = on_connect
# client.on_disconnect = on_disconnect

client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
client.username_pw_set(cfg["username"], cfg["password"])
client.connect(cfg["broker"], cfg["port"], 60)
client.loop_start()  # Start background thread


# Mouse event handlers
def on_move(x, y):
    client.publish(TOPIC, json.dumps({'type': 'mouse_move', 'x': x, 'y': y}))

def on_click(x, y, button, pressed):
    action = 'down' if pressed else 'up'
    client.publish(TOPIC, json.dumps({
        'type': 'mouse_click',
        'x': x,
        'y': y,
        'button': button.name,
        'action': action
    }))

def on_scroll(x, y, dx, dy):
    client.publish(TOPIC, json.dumps({
        'type': 'mouse_scroll',
        'dx': dx,
        'dy': dy
    }))

# Keyboard event handlers
def on_press(key):
    try:
        k = key.char
    except AttributeError:
        k = key.name
    client.publish(TOPIC, json.dumps({
        'type': 'keyboard',
        'key': k,
        'action': 'press'
    }))

def on_release(key):
    try:
        k = key.char
    except AttributeError:
        k = key.name
    client.publish(TOPIC, json.dumps({
        'type': 'keyboard',
        'key': k,
        'action': 'release'
    }))

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

print("Publisher started. Press Ctrl+C to exit.")
try:
    while True:
        pass
except KeyboardInterrupt:
    mouse_listener.stop()
    keyboard_listener.stop()
    client.loop_stop()