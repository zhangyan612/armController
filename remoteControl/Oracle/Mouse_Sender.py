import time
import json
import math
import oci
from base64 import b64encode
from pynput import mouse, keyboard

# OCI Config
ociMessageEndpoint = "https://cell-1.streaming.us-chicago-1.oci.oraclecloud.com"
ociStreamOcid = "ocid1.stream.oc1.us-chicago-1.amaaaaaaoa2s45aam2sr7zfgu34z365rgfkeuch5qule2b2dbbpztvoudq6q"  
ociConfigFilePath = "C:\\Users\\yanzh\\.oci\\config"
ociProfileName = "DEFAULT"


# Setup OCI Stream Client
config = oci.config.from_file(ociConfigFilePath, ociProfileName)
stream_client = oci.streaming.StreamClient(config, service_endpoint=ociMessageEndpoint)

def send_event(event_type, data):
    payload = {'type': event_type, **data}
    encoded_value = b64encode(json.dumps(payload).encode()).decode()
    message = oci.streaming.models.PutMessagesDetailsEntry(value=encoded_value)
    messages = oci.streaming.models.PutMessagesDetails(messages=[message])
    stream_client.put_messages(ociStreamOcid, messages)

# Track only meaningful mouse movement
MOVE_THRESHOLD = 10
current_pos = {'x': 0, 'y': 0}
last_sent_pos = {'x': 0, 'y': 0}

def distance(p1, p2):
    return math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])

# Mouse callbacks
def on_move(x, y):
    global current_pos, last_sent_pos
    current_pos = {'x': int(x), 'y': int(y)}
    if distance(current_pos, last_sent_pos) >= MOVE_THRESHOLD:
        print(f"Mouse moved to: {current_pos}")
        send_event('move', current_pos)
        last_sent_pos = current_pos.copy()

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

# Keep alive
while True:
    time.sleep(2)
