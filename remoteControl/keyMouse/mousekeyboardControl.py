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

# Track modifier keys
pressed_modifiers = {
    'ctrl': False,
    'shift': False,
    'alt': False,
    'win': False
}

# Mapping for control characters to key names
CONTROL_CHAR_MAP = {
    '\x01': 'a',  # Ctrl+A
    '\x03': 'c',  # Ctrl+C
    '\x16': 'v',  # Ctrl+V
    '\x1a': 'z',  # Ctrl+Z
    '\x08': 'h',  # Ctrl+H (Backspace)
    '\x09': 'i',  # Ctrl+I (Tab)
    '\x0a': 'j',  # Ctrl+J (Enter)
    '\x0d': 'm',  # Ctrl+M (Enter)
    '\x1b': '[',  # Escape
}

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
        # print(f"Mouse moved to: {current_pos}")
        send_move('move', current_pos)
        if (current_pos['x'] < 0):
            current_pos['x'] = 0
        if (current_pos['y'] < 0):
            current_pos['y'] = 0
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
    global pressed_modifiers
    
    # Update modifier state
    if key == keyboard.Key.ctrl or key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
        pressed_modifiers['ctrl'] = True
    elif key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
        pressed_modifiers['shift'] = True
    elif key == keyboard.Key.alt or key == keyboard.Key.alt_l or key == keyboard.Key.alt_r:
        pressed_modifiers['alt'] = True
    elif key == keyboard.Key.cmd or key == keyboard.Key.cmd_l or key == keyboard.Key.cmd_r:
        pressed_modifiers['win'] = True
    
    try:
        k = key.char
        # Handle control characters
        if k in CONTROL_CHAR_MAP:
            k = CONTROL_CHAR_MAP[k]
        
        command = k.upper() * 2
        send_event('keyboard', {'key': command, 'action': 'press'})
        keyboard_dev.send_data(command)
    except AttributeError:
        # Handle special keys
        k = key.name
        print('exception')
        print(k)

        # Handle modifier keys differently
        if k in ['ctrl', 'ctrl_l', 'ctrl_r', 
                 'shift', 'shift_l', 'shift_r', 
                 'alt', 'alt_l', 'alt_r', 
                 'cmd', 'cmd_l', 'cmd_r']:
            # Modifier keys are handled by state tracking
            return
        
        # Map key names to our command codes
        KEY_MAPPING = {
            # Navigation keys
            'insert': 'IN', 'home': 'HM', 'page_up': 'PU', 'delete': 'DE',
            'end': 'EN', 'page_down': 'PD', 'right': 'RA', 'left': 'LA',
            'down': 'DA', 'up': 'UA',
            
            # Function keys
            'f1': 'F1', 'f2': 'F2', 'f3': 'F3', 'f4': 'F4', 'f5': 'F5',
            'f6': 'F6', 'f7': 'F7', 'f8': 'F8', 'f9': 'F9', 'f10': 'F10',
            'f11': 'F11', 'f12': 'F12',
            
            # System keys
            'print_screen': 'PS', 'scroll_lock': 'SL', 'pause': 'PB',
            
            # Special keys
            'enter': 'EN', 'backspace': 'BA', 'tab': 'TB', 'space': 'SP',
            'caps_lock': 'CL', 'esc': 'ES',
            
            # Numpad keys
            'num_lock': 'NL', 
            'keypad_enter': 'KE',
            'keypad_multiply': 'KM',
            'keypad_divide': 'KD',
            'keypad_add': 'K+',
            'keypad_subtract': 'K-',
            'keypad_decimal': 'K.',
            'keypad_insert': 'K0',
            'keypad_end': 'K1',
            'keypad_down': 'K2',
            'keypad_page_down': 'K3',
            'keypad_left': 'K4',
            'keypad_begin': 'K5',
            'keypad_right': 'K6',
            'keypad_home': 'K7',
            'keypad_up': 'K8',
            'keypad_page_up': 'K9',
            'keypad_delete': 'K.',
            
            # Symbols
            '-': 'MI', '=': 'EQ', '[': 'LB', ']': 'RB', '\\': 'BS',
            ';': 'SC', "'": 'SQ', '`': 'GR', ',': 'CM', '.': 'DT', '/': 'SL'
        }
        command = KEY_MAPPING.get(k, 'NU')
        send_event('keyboard', {'key': command, 'action': 'press'})
        keyboard_dev.send_data(command)

def on_release(key):
    global pressed_modifiers
    
    # Update modifier state
    if key == keyboard.Key.ctrl or key == keyboard.Key.ctrl_l or key == keyboard.Key.ctrl_r:
        pressed_modifiers['ctrl'] = False
    elif key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
        pressed_modifiers['shift'] = False
    elif key == keyboard.Key.alt or key == keyboard.Key.alt_l or key == keyboard.Key.alt_r:
        pressed_modifiers['alt'] = False
    elif key == keyboard.Key.cmd or key == keyboard.Key.cmd_l or key == keyboard.Key.cmd_r:
        pressed_modifiers['win'] = False
    
    send_event('keyboard', {'key': key.name if hasattr(key, 'name') else str(key), 'action': 'release'})
    keyboard_dev.release()

# Start listeners

mouse_listener = mouse.Listener(on_move=on_move, on_click=on_click)
keyboard_listener = keyboard.Listener(on_press=on_press, on_release=on_release)

mouse_listener.start()
keyboard_listener.start()

try:
    print("Starting input capture... Press Ctrl+C to exit.")
    while True:
        time.sleep(0.1)  # Reduced sleep for better responsiveness
except KeyboardInterrupt:
    print("\nExiting...")
    mouse_listener.stop()
    keyboard_listener.stop()