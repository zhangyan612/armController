from serial import Serial
import time
import json
from pynput import mouse, keyboard
import math
from ch9329 import Keyboard, Mouse


KEY_MAPPING = {
    'shift': 'shift_left',
    'shift_r': 'shift_right', 
    'ctrl_l': 'ctrl_left',
    'ctrl_r': 'ctrl_right',
    'cmd': 'win',
    'alt_l': 'alt_right', 
    'alt_gr': 'alt_left'
}

class CH9329:
    ser: Serial = None

    def __init__(self, port: str, baudrate: int, timeout: int = 1, screenx: int = 1920, screeny: int = 1080):
        """
        CH9329控制的集合
        :param port: COM端口
        :param baudrate: 波特率
        :param timeout: 超时时间
        :param screenx: 屏幕X轴
        :param screeny: 屏幕Y轴
        """
        self.ser = Serial(port, baudrate, timeout=timeout)
        self.keyboard = Keyboard(self.ser)
        self.mouse = Mouse(self.ser, screenx, screeny)

    def close(self):
        """释放串口"""
        self.ser.close()


ch9329 = CH9329("COM12", 9600, timeout=1, screenx=1920, screeny=1080)

# ch9329.mouse.absolute_move(111, 333)

# Position cache
current_pos = {'x': 0, 'y': 0}
current_modifier = ''

# Track pressed keys and modifiers
pressed_keys = set()
pressed_modifiers = set()

def send_event(event_type, data):
    payload = {'type': event_type, **data}

    if payload['type'] == 'keyboard':
        print('keyboard event')
        print(payload)
        if payload['key'] == '\x01':
            print('ctrl A pressed')
            ch9329.keyboard.press("a", modifiers=['ctrl'])
        elif payload['key'] == '\x03':
            print('ctrl C pressed')
            ch9329.keyboard.press("c", modifiers=['ctrl'])
        elif payload['key'] == '\x16':
            print('ctrl V pressed')
            ch9329.keyboard.press("v", modifiers=['ctrl'])
        elif payload['key'] == '\x1a':
            print('ctrl Z pressed')
            ch9329.keyboard.press("z", modifiers=['ctrl'])
        elif payload['key'] == '\x18':
            print('ctrl X pressed')
            ch9329.keyboard.press("x", modifiers=['ctrl'])
        elif payload['key'] == '\x06':
            print('ctrl F pressed')
            ch9329.keyboard.press("f", modifiers=['ctrl'])
        elif payload['key'] == '\x13':
            print('ctrl S pressed')
            ch9329.keyboard.press("s", modifiers=['ctrl'])
        elif payload['key'] == None:
            ch9329.keyboard.press("/", modifiers=['ctrl_right'])
        else:
            ch9329.keyboard.press(payload['key'])

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
        ch9329.mouse.absolute_move(current_pos['x'], current_pos['y'])
        last_sent_pos = current_pos.copy()

def send_mouse_button(button, action):
    if button == 'left':
        if action == 'down':
            ch9329.mouse.press("left")
        elif action == 'up':
            ch9329.mouse.release()
    elif button == 'right':
        if action == 'down':
            ch9329.mouse.press("right")
        elif action == 'up':
            ch9329.mouse.release()
    elif button == 'middle':
        if action == 'down':
            ch9329.mouse.press("center")
        elif action == 'up':
            ch9329.mouse.release()

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
    ch9329.mouse.absolute_move(int(x), int(y))
    send_mouse_button(button.name, 'down' if pressed else 'up')

def on_scroll(x, y, dx, dy):
    """Handle mouse scroll events"""
    # send_event('scroll', {
    #     'x': int(x),
    #     'y': int(y),
    #     'dx': int(dx),
    #     'dy': int(dy)
    # })
    
    # Update mouse position first
    ch9329.mouse.wheel(dy)


# Keyboard callbacks
# Update the keyboard callbacks
def on_press(key):
    try:
        # Try to get character
        k = key.char
        print(f"Key pressed: {k}")
        # Send as normal key
        send_event('keyboard', {'key': k, 'action': 'press'})
        # if k == '→':
        #     print('modifier key pressed')
        #     print(f'current modifer {current_modifier}')
        #     ch9329.keyboard.press("c", modifiers=[current_modifier])
        # else:
        #     ch9329.keyboard.press(k)

    except AttributeError:
        # Handle special keys
        k = key.name
        if k in KEY_MAPPING:
            print(f"Modifier key pressed: {k}")
            current_modifier = KEY_MAPPING[k]
            # ch9329.keyboard.press(command)
        else:
            print(f"Special key pressed: {k}")
            ch9329.keyboard.press(k)
        

def on_release(key):
    # Always release - this clears any held modifier keys
    ch9329.keyboard.release()
    # send_event('keyboard', {'key': k, 'action': 'release'} 'release')

    # Optional: you might want to track which keys are being released
    try:
        k = key.char
        print(f"Key released: {k}")
    except AttributeError:
        print(f"Special key released: {key}")
        current_modifier = ''

    
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