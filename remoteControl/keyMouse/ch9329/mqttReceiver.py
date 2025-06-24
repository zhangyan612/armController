import time
import json
import paho.mqtt.client as mqtt
from serial import Serial
from ch9329 import Keyboard, Mouse
import os

# MQTT Configuration
TOPIC = "remote_keyboard_mouse"
MQTT_CONFIG_PATH = "mqtt_config.json"

# Key mapping for special keys
KEY_MAPPING = {
    'shift': 'shift_left',
    'shift_r': 'shift_right', 
    'ctrl_l': 'ctrl_left',
    'ctrl_r': 'ctrl_right',
    'cmd': 'win',
    'alt_l': 'alt_right', 
    'alt_gr': 'alt_left'
}

class CH9329Controller:
    def __init__(self, port="COM12", baudrate=9600, screenx=1920, screeny=1080):
        self.ser = Serial(port, baudrate, timeout=1)
        self.keyboard = Keyboard(self.ser)
        self.mouse = Mouse(self.ser, screenx, screeny)
        self.current_pos = {'x': 0, 'y': 0}
        
    def handle_mouse_move(self, data):
        self.mouse.absolute_move(data['x'], data['y'])
        self.current_pos = {'x': data['x'], 'y': data['y']}
    
    def handle_mouse_click(self, data):
        button = data['button']
        action = data['action']
        
        if button == 'left':
            if action == 'down': self.mouse.press("left")
            else: self.mouse.release()
        elif button == 'right':
            if action == 'down': self.mouse.press("right")
            else: self.mouse.release()
        elif button == 'middle':
            if action == 'down': self.mouse.press("center")
            else: self.mouse.release()
    
    def handle_mouse_scroll(self, data):
        self.mouse.wheel(data['dy'])
    
    def handle_key_press(self, data):
        key = data['key']
        
        # Handle special key combinations
        special_combos = {
            '\x01': ("a", ['ctrl']),  # Ctrl+A
            '\x03': ("c", ['ctrl']),  # Ctrl+C
            '\x16': ("v", ['ctrl']),  # Ctrl+V
            '\x1a': ("z", ['ctrl']),  # Ctrl+Z
            '\x18': ("x", ['ctrl']),  # Ctrl+X
            '\x06': ("f", ['ctrl']),  # Ctrl+F
            '\x13': ("s", ['ctrl']),  # Ctrl+S
        }
        
        if key in special_combos:
            k, mods = special_combos[key]
            self.keyboard.press(k, modifiers=mods)
        elif key == 'f1':
            self.keyboard.send(("ctrl", "alt", "del", "", "", ""), modifiers=[])
        elif key == 'f2':
            self.keyboard.write("906120")
        elif key == 'f3':
            self.keyboard.write("ZYmeng94")
        else:
            # Map special keys
            mapped_key = KEY_MAPPING.get(key, key)
            self.keyboard.press(mapped_key)
    
    def handle_key_release(self, data):
        self.keyboard.release()

def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        controller = userdata["controller"]
        
        if data['type'] == 'mouse_move':
            controller.handle_mouse_move(data)
        elif data['type'] == 'mouse_click':
            controller.handle_mouse_click(data)
        elif data['type'] == 'mouse_scroll':
            controller.handle_mouse_scroll(data)
        elif data['type'] == 'key_press':
            controller.handle_key_press(data)
        elif data['type'] == 'key_release':
            controller.handle_key_release(data)
            
    except Exception as e:
        print(f"Error handling message: {str(e)}")

def load_config():
    with open(MQTT_CONFIG_PATH, "r") as f:
        return json.load(f)

if __name__ == "__main__":
    # Setup CH9329 controller
    controller = CH9329Controller()
    
    # MQTT Setup
    cfg = load_config()
    client = mqtt.Client(userdata={"controller": controller})
    client.on_message = on_message
    
    client.connect(cfg["broker"], cfg["port"], 60)
    client.subscribe(TOPIC)
    
    print("Input receiver started. Listening for events...")
    client.loop_forever()