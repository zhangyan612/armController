import time
import json
from pynput import mouse, keyboard
import math
import ch9329Comm
import serial

# Your existing KEY_MAPPING dictionary
KEY_MAPPING = {
    # Digits
    '1': '11', '2': '22', '3': '33', '4': '44', '5': '55',
    '6': '66', '7': '77', '8': '88', '9': '99', '0': '00',
    
    # Special characters
    '-': 'MI', '=': 'EQ', '[': 'LB', ']': 'RB', '\\': 'BS',
    ';': 'SC', "'": 'SQ', '`': 'GR', ',': 'CM', '.': 'DT', '/': 'SL',
    
    # Function keys
    'f1': 'F1', 'f2': 'F2', 'f3': 'F3', 'f4': 'F4', 'f5': 'F5',
    'f6': 'F6', 'f7': 'F7', 'f8': 'F8', 'f9': 'F9', 'f10': 'F10',
    'f11': 'F11', 'f12': 'F12',
    
    # System keys
    'print_screen': 'PS', 'scroll_lock': 'SL', 'pause': 'PB',
    
    # Navigation keys
    'insert': 'IN', 'home': 'HM', 'page_up': 'PU', 'delete': 'DE',
    'end': 'END', 'page_down': 'PD', 'right': 'RA', 'left': 'LA',
    'down': 'DA', 'up': 'UA',
    
    # Common keys
    'num_lock': 'NL', 
    'enter': 'EN', 'backspace': 'BA', 'tab': 'TB', 'space': 'SP',
    'caps_lock': 'CL', 'escape': 'ES',
    
    # Numpad keys
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
    
    # Special keys that are not modifiers
    'menu': 'KA',  # Menu/Apps key
    
    # Modifier keys - mapped to control_button_hex_dict keys
    'shift': 'L_SHIFT',
    'shift_r': 'R_SHIFT', 
    'ctrl_l': 'L_CTRL',
    'ctrl_r': 'R_CTRL',
    'cmd': 'L_WIN',
    'alt_l': 'L_ALT', 
    'alt_gr': 'R_ALT'
}

# Setup serial and ch9329
serial.ser = serial.Serial('COM13', 9600)

mouse_dev = ch9329Comm.mouse.DataComm(1920, 1080)
keyboard_dev = ch9329Comm.keyboard.DataComm()

# Position cache
current_pos = {'x': 0, 'y': 0}

# Track pressed keys and modifiers
pressed_keys = set()
pressed_modifiers = set()

def send_event(event_type, data):
    payload = {'type': event_type, **data}
    print(payload)

def send_move(event_type, data):
    payload = {'type': event_type, **data}
    print(payload)

MOVE_THRESHOLD = 10  # Only send if movement > 10 pixels
last_sent_pos = {'x': 0, 'y': 0}

def distance(p1, p2):
    return math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])

# Mouse callbacks
def on_move(x, y):
    global current_pos, last_sent_pos
    if x < 0:
        x = 0
    if y < 0:
        y = 0

    current_pos = {'x': int(x), 'y': int(y)}
    if distance(current_pos, last_sent_pos) >= MOVE_THRESHOLD:
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
    if x < 0:
        x = 0
    if y < 0:
        y = 0

    send_event('mouse', {
        'x': int(x),
        'y': int(y),
        'button': button.name,
        'action': 'down' if pressed else 'up'
    })
    mouse_dev.send_data_absolute(int(x), int(y))
    send_mouse_button(button.name, 'down' if pressed else 'up')

def on_scroll(x, y, dx, dy):
    """Handle mouse scroll events"""
    send_event('scroll', {
        'x': int(x),
        'y': int(y),
        'dx': int(dx),
        'dy': int(dy)
    })
    
    # Update mouse position first
    mouse_dev.send_data_absolute(int(x), int(y))
    
    # Handle vertical scrolling
    # if dy > 0:
    #     # Scroll up
    #     mouse_dev.send_data_relatively(0, 0, 'WU')  # Assuming 'WU' for wheel up
    # elif dy < 0:
    #     # Scroll down  
    #     mouse_dev.send_data_relatively(0, 0, 'WD')  # Assuming 'WD' for wheel down

# Enhanced keyboard callbacks
def on_press(key):
    global pressed_keys, pressed_modifiers
    
    try:
        # Try to get character
        k = key.char
        print(f"Key pressed: {k}")
        
        # Handle digits and special characters
        if k in KEY_MAPPING:
            command = KEY_MAPPING[k]
        else:
            # Handle letters
            command = k.upper() * 2
        
        pressed_keys.add(command)
        send_current_key_state()
        
    except AttributeError:
        # Handle special keys
        k = key.name
        print(f"Special key pressed: {k}")
        
        # Check if it's a modifier key
        modifier_keys = ['shift', 'shift_r', 'ctrl_l', 'ctrl_r', 'cmd', 'alt_l', 'alt_gr']
        
        if k in modifier_keys:
            # Map to the modifier name expected by the enhanced library
            modifier_map = {
                'shift': 'shift_left',
                'shift_r': 'shift_right',
                'ctrl_l': 'ctrl_left', 
                'ctrl_r': 'ctrl_right',
                'cmd': 'win_left',
                'alt_l': 'alt_left',
                'alt_gr': 'alt_right'
            }
            modifier_name = modifier_map.get(k, k)
            pressed_modifiers.add(modifier_name)
            send_current_key_state()
        else:
            # Handle other special keys normally
            command = KEY_MAPPING.get(k, 'NU')
            if command != 'NU':
                pressed_keys.add(command)
                send_current_key_state()

def on_release(key):
    global pressed_keys, pressed_modifiers
    
    try:
        # Try to get character
        k = key.char
        print(f"Key released: {k}")
        
        # Handle digits and special characters
        if k in KEY_MAPPING:
            command = KEY_MAPPING[k]
        else:
            # Handle letters
            command = k.upper() * 2
        
        pressed_keys.discard(command)
        send_current_key_state()
        
    except AttributeError:
        # Handle special keys
        k = key.name
        print(f"Special key released: {k}")
        
        # Check if it's a modifier key
        modifier_keys = ['shift', 'shift_r', 'ctrl_l', 'ctrl_r', 'cmd', 'alt_l', 'alt_gr']
        
        if k in modifier_keys:
            # Map to the modifier name expected by the enhanced library
            modifier_map = {
                'shift': 'shift_left',
                'shift_r': 'shift_right',
                'ctrl_l': 'ctrl_left', 
                'ctrl_r': 'ctrl_right',
                'cmd': 'win_left',
                'alt_l': 'alt_left',
                'alt_gr': 'alt_right'
            }
            modifier_name = modifier_map.get(k, k)
            pressed_modifiers.discard(modifier_name)
            send_current_key_state()
        else:
            # Handle other special keys normally
            command = KEY_MAPPING.get(k, 'NU')
            if command != 'NU':
                pressed_keys.discard(command)
                send_current_key_state()

def send_current_key_state():
    """Send the current state of all pressed keys and modifiers"""
    # Convert sets to lists and limit to 6 keys
    keys_list = list(pressed_keys)[:6]
    modifiers_list = list(pressed_modifiers)
    
    # Create key tuple with up to 6 keys
    key_tuple = [""] * 6
    for i, key in enumerate(keys_list):
        key_tuple[i] = key
    
    # Send the key combination
    print(f"Sending keys: {key_tuple}, modifiers: {modifiers_list}")
    keyboard_dev.send_multiple_keys(tuple(key_tuple), modifiers_list)
    
    send_event('keyboard_state', {
        'keys': keys_list,
        'modifiers': modifiers_list,
        'action': 'update'
    })

# Start listeners
mouse_listener = mouse.Listener(
    on_move=on_move, 
    on_click=on_click,
    on_scroll=on_scroll
)
keyboard_listener = keyboard.Listener(on_press=on_press, on_release=on_release)

mouse_listener.start()
keyboard_listener.start()

try:
    print("Starting enhanced input capture... Press Ctrl+C to exit.")
    print("Now supports:")
    print("- Multi-key combinations (up to 6 keys)")
    print("- Modifier keys (Ctrl, Alt, Shift, Win)")
    print("- Mouse scroll wheel")
    print("- Middle mouse button")
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nExiting...")
    mouse_listener.stop()
    keyboard_listener.stop()