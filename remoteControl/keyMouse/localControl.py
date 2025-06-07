import time
from pynput import mouse, keyboard
import ch9329Comm
import serial

# Setup serial and ch9329
serial.ser = serial.Serial('COM10', 9600)
mouse_dev = ch9329Comm.mouse.DataComm(1920, 1080)
keyboard_dev = ch9329Comm.keyboard.DataComm()

# Position cache
current_pos = {'x': 0, 'y': 0}
last_sent_pos = {'x': 0, 'y': 0}
MOVE_THRESHOLD = 10  # Only send if movement > 10 pixels

# Comprehensive key mapping
KEY_MAPPING = {
    # Modifier keys
    'ctrl': 'LCTRL', 'control': 'LCTRL',
    'shift': 'LSHIFT',
    'alt': 'LALT',
    'ctrl_l': 'LCTRL', 'ctrl_r': 'RCTRL',
    'shift_l': 'LSHIFT', 'shift_r': 'RSHIFT',
    'alt_l': 'LALT', 'alt_r': 'RALT',
    'cmd': 'LSUPER', 'command': 'LSUPER', 'super': 'LSUPER',
    
    # Special keys
    'enter': 'ENTER', 'return': 'ENTER',
    'esc': 'ESC', 'escape': 'ESC',
    'backspace': 'BACKSPACE',
    'tab': 'TAB',
    'caps_lock': 'CAPSLOCK',
    'space': 'SPACE',
    'delete': 'DELETE',
    'insert': 'INSERT',
    'home': 'HOME',
    'end': 'END',
    'page_up': 'PGUP', 'prior': 'PGUP',
    'page_down': 'PGDN', 'next': 'PGDN',
    
    # Arrow keys
    'up': 'UP',
    'down': 'DOWN',
    'left': 'LEFT',
    'right': 'RIGHT',
    
    # Function keys
    'f1': 'F1', 'f2': 'F2', 'f3': 'F3', 'f4': 'F4',
    'f5': 'F5', 'f6': 'F6', 'f7': 'F7', 'f8': 'F8',
    'f9': 'F9', 'f10': 'F10', 'f11': 'F11', 'f12': 'F12',
    
    # Special characters
    '`': '`', '~': '~',
    '!': '!', '@': '@', '#': '#', '$': '$', '%': '%',
    '^': '^', '&': '&', '*': '*', '(': '(', ')': ')',
    '-': '-', '_': '_', '=': '=', '+': '+',
    '[': '[', ']': ']', '{': '{', '}': '}',
    '\\': '\\', '|': '|',
    ';': ';', ':': ':', "'": "'", '"': '"',
    ',': ',', '<': '<', '.': '.', '>': '>', '/': '/', '?': '?',
    
    # Numeric keys
    '0': '0', '1': '1', '2': '2', '3': '3', '4': '4',
    '5': '5', '6': '6', '7': '7', '8': '8', '9': '9',
}

# Mouse button actions
def send_mouse_button(button, action):
    if button == 'left':
        if action == 'down':
            mouse_dev.press_left()
        elif action == 'up':
            mouse_dev.release_left()
    elif button == 'right':
        if action == 'down':
            mouse_dev.press_right()
        elif action == 'up':
            mouse_dev.release_right()
    elif button == 'middle':
        if action == 'down':
            mouse_dev.press_middle()
        elif action == 'up':
            mouse_dev.release_middle()

# Keyboard actions
def send_keyboard_action(key, action):
    # Normalize key name
    key = key.lower()
    
    # Map to CH9329 compatible name
    mapped_key = KEY_MAPPING.get(key, key.upper())
    
    if action == 'press':
        try:
            # Handle modifiers
            if mapped_key in ['LCTRL', 'RCTRL']:
                keyboard_dev.press_ctrl()
            elif mapped_key in ['LSHIFT', 'RSHIFT']:
                keyboard_dev.press_shift()
            elif mapped_key in ['LALT', 'RALT']:
                keyboard_dev.press_alt()
            elif mapped_key in ['LSUPER']:
                keyboard_dev.press_super()
            else:
                keyboard_dev.press(mapped_key)
        except Exception as e:
            print(f"Key press error: {e}")
    
    elif action == 'release':
        try:
            # Handle modifiers
            if mapped_key in ['LCTRL', 'RCTRL']:
                keyboard_dev.release_ctrl()
            elif mapped_key in ['LSHIFT', 'RSHIFT']:
                keyboard_dev.release_shift()
            elif mapped_key in ['LALT', 'RALT']:
                keyboard_dev.release_alt()
            elif mapped_key in ['LSUPER']:
                keyboard_dev.release_super()
            else:
                keyboard_dev.release(mapped_key)
        except Exception as e:
            print(f"Key release error: {e}")

# Mouse callbacks
def on_move(x, y):
    global current_pos, last_sent_pos
    current_pos = {'x': int(x), 'y': int(y)}
    
    # Calculate distance moved
    dx = abs(current_pos['x'] - last_sent_pos['x'])
    dy = abs(current_pos['y'] - last_sent_pos['y'])
    
    # Only send if movement exceeds threshold
    if dx >= MOVE_THRESHOLD or dy >= MOVE_THRESHOLD:
        print(f"Sending absolute mouse position: ({current_pos['x']}, {current_pos['y']})")
        mouse_dev.send_data_absolutely(current_pos['x'], current_pos['y'])
        last_sent_pos = current_pos.copy()

def on_click(x, y, button, pressed):
    button_name = button.name
    action = 'down' if pressed else 'up'
    print(f"Mouse {button_name} {action} at ({int(x)}, {int(y)})")
    send_mouse_button(button_name, action)

# Keyboard callbacks
def on_press(key):
    try:
        k = key.char
    except AttributeError:
        k = key.name
    
    print(f"Key press: {k}")
    send_keyboard_action(k, 'press')

def on_release(key):
    try:
        k = key.char
    except AttributeError:
        k = key.name
    
    print(f"Key release: {k}")
    send_keyboard_action(k, 'release')

# Start listeners
mouse_listener = mouse.Listener(on_move=on_move, on_click=on_click)
keyboard_listener = keyboard.Listener(on_press=on_press, on_release=on_release)

mouse_listener.start()
keyboard_listener.start()

# Keep running
try:
    print("Starting input capture... Press Ctrl+C to exit.")
    while True:
        time.sleep(0.1)  # Reduced sleep for better responsiveness
except KeyboardInterrupt:
    print("\nExiting...")
    serial.ser.close()
    mouse_listener.stop()
    keyboard_listener.stop()