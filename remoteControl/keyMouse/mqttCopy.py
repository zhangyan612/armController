import time
import json
from pynput import mouse, keyboard
import math
import ch9329Comm
import serial

# Add this mapping helper in the main code
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
        # print(f"Mouse moved to: {current_pos}")
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


def send_mouse_scroll(direction, delta):
    """Send mouse scroll events to CH9329"""
    # Convert scroll delta to appropriate scroll commands
    # Positive delta = scroll up, negative delta = scroll down
    if direction == 'up':
        print(f"Scrolling up: {delta}")
        # not working, library support questionable
        mouse_dev.send_data_relatively(0, 0, 'NU')  # Assuming 'SU' for scroll up
    elif direction == 'down':
        print(f"Scrolling down: {delta}")
        # not working, library support questionable
        mouse_dev.send_data_relatively(0, 0, 'CE')  # Assuming 'SD' for scroll down


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
# Update the keyboard callbacks
def on_press(key):
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
        
        # Send as normal key
        send_event('keyboard', {'key': command, 'action': 'press'})

        if command == '\x03\x03':
            print("Ctrl+C detected")
            keyboard_dev.send_multiple_keys(("L_CTRL", "CC", "", "", "", ""))
            # keyboard_dev.send_data('','0x03') # 同时按下ctrl+shift
        elif command == '\x16\x16':
            print("Ctrl+V detected")
            # keyboard_dev.send_data('','0x16')
            keyboard_dev.send_multiple_keys(("L_CTRL", "VV", "", "", "", ""))

        elif command == '\x01\x01':
            print("Ctrl+A detected")
            keyboard_dev.send_data('','0x01')
        else:
            keyboard_dev.send_data(command)
        
    except AttributeError:
        # Handle special keys
        k = key.name
        print(f"Special key pressed: {k}")
        
        # Check if it's a modifier key - map to control_button_hex_dict keys
        modifier_keys = ['shift', 'shift_r', 'ctrl_l', 'ctrl_r', 'cmd', 'alt_l', 'alt_gr']
        
        if k in modifier_keys:
            ctrl_command = KEY_MAPPING.get(k, 'NULL')
            send_event('keyboard', {'key': ctrl_command, 'action': 'press'})
            # For modifier keys, send with control parameter
            keyboard_dev.send_data('', ctrl_command)
        else:
            # Handle other special keys normally
            command = KEY_MAPPING.get(k, 'NU')
            send_event('keyboard', {'key': command, 'action': 'press'})
            keyboard_dev.send_data(command)


def on_release(key):
    # Always release - this clears any held modifier keys
    keyboard_dev.release()
    
    # Optional: you might want to track which keys are being released
    try:
        k = key.char if key.char else key.name
        print(f"Key released: {k}")
    except AttributeError:
        print(f"Special key released: {key.name}")


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
    
    # Handle vertical scrolling (most common)
    if dy > 0:
        # Scroll up
        for _ in range(abs(int(dy))):
            send_mouse_scroll('up', dy)
    elif dy < 0:
        # Scroll down  
        for _ in range(abs(int(dy))):
            send_mouse_scroll('down', dy)
    
    # Handle horizontal scrolling if needed
    if dx > 0:
        # Scroll right - you may need to implement this based on your CH9329 capabilities
        print(f"Horizontal scroll right: {dx}")
    elif dx < 0:
        # Scroll left
        print(f"Horizontal scroll left: {dx}")


# Start listeners

mouse_listener = mouse.Listener(on_move=on_move, on_click=on_click, on_scroll=on_scroll)
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
