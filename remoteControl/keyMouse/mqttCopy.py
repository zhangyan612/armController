import time
import json
from pynput import mouse, keyboard
import math
import ch9329Comm
import serial

# Setup serial and ch9329
serial.ser = serial.Serial('COM10', 9600)

mouse_dev = ch9329Comm.mouse.DataComm(1920, 1080)
keyboard_dev = ch9329Comm.keyboard.DataComm()

# Position cache
current_pos = {'x': 0, 'y': 0}

def send_event(event_type, data):
    payload = {'type': event_type, **data}
    print(payload)

def send_move(event_type, data):
    payload = {'type': event_type, **data}
    print(payload)
    

MOVE_THRESHOLD = 10  # Only send if movement > 5 pixels

last_sent_pos = {'x': 0, 'y': 0}

def distance(p1, p2):
    return math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])

# Mouse callbacks
def on_move(x, y):
    global current_pos, last_sent_pos
    current_pos = {'x': int(x), 'y': int(y)}
    if distance(current_pos, last_sent_pos) >= MOVE_THRESHOLD:
        print(f"Mouse moved to: {current_pos}")
        # send_move('move', current_pos)
        mouse_dev.send_data_absolute(current_pos['x'], current_pos['y'])
        last_sent_pos = current_pos.copy()

def send_mouse_button(button, action):
    if button == 'left':
        if action == 'down':
            mouse_dev.send_data_relatively(0, 0, 'LE')
        elif action == 'up':
            mouse_dev.send_data_relatively(0, 0, 'NU')
    elif button == 'right':
        if action == 'down':
            mouse_dev.send_data_relatively(0, 0, 'RI')
        elif action == 'up':
            mouse_dev.send_data_relatively(0, 0, 'NU')
    elif button == 'middle':
        if action == 'down':
            mouse_dev.send_data_relatively(0, 0, 'CE')
        elif action == 'up':
            mouse_dev.send_data_relatively(0, 0, 'NU')

def on_click(x, y, button, pressed):
    send_event('mouse', {
        'x': int(x),
        'y': int(y),
        'button': button.name,
        'action': 'down' if pressed else 'up'
    })
    mouse_dev.send_data_absolute(int(x), int(y))
    send_mouse_button(button.name, 'down' if pressed else 'up')

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

mouse_listener = mouse.Listener(on_move=on_move, on_click=on_click)
keyboard_listener = keyboard.Listener(on_press=on_press, on_release=on_release)

mouse_listener.start()
keyboard_listener.start()


# while True:
    
    
#     time.sleep(2)
try:
    print("Starting input capture... Press Ctrl+C to exit.")
    while True:
        time.sleep(0.1)  # Reduced sleep for better responsiveness
except KeyboardInterrupt:
    print("\nExiting...")
    mouse_listener.stop()
    keyboard_listener.stop()
