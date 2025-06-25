from serial import Serial
import time
import json
import threading
from pynput import mouse, keyboard
import math
from ch9329 import Keyboard, Mouse
import paho.mqtt.client as mqtt
import os
import logging
import ssl
import paho

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('input_controller')

# MQTT Configuration
MQTT_CONFIG_PATH = "mqtt_config.json"
CONTROL_MODE_TOPIC = "control_mode"
REMOTE_INPUT_TOPIC = "remote_keyboard_mouse"

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

# Global variables
control_mode = "local"  # "local" or "remote"
mqtt_client = None
mouse_listener = None
keyboard_listener = None
last_sent_pos = {'x': 0, 'y': 0}
MOVE_THRESHOLD = 10

class CH9329:
    ser: Serial = None

    def __init__(self, port: str, baudrate: int, timeout: int = 1, screenx: int = 1920, screeny: int = 1080):
        try:
            self.ser = Serial(port, baudrate, timeout=timeout)
            self.keyboard = Keyboard(self.ser)
            self.mouse = Mouse(self.ser, screenx, screeny)
            logger.info(f"CH9329 initialized on {port}")
        except Exception as e:
            logger.error(f"Failed to initialize CH9329: {str(e)}")
            raise
        
    def close(self):
        """释放串口"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Serial port closed")

# Initialize CH9329 controller
ch9329 = CH9329("COM12", 9600, timeout=1, screenx=1920, screeny=1080)

def distance(p1, p2):
    return math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])

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

# Mouse callbacks
def on_move(x, y):
    global last_sent_pos
    if control_mode != "local":
        return
        
    if x < 0:
        x = 0
    if y < 0:
        y = 0

    current_pos = {'x': int(x), 'y': int(y)}
    if distance(current_pos, last_sent_pos) >= MOVE_THRESHOLD:
        ch9329.mouse.absolute_move(current_pos['x'], current_pos['y'])
        last_sent_pos = current_pos.copy()
        logger.debug(f"Local mouse moved to ({current_pos['x']}, {current_pos['y']})")

def on_click(x, y, button, pressed):
    if control_mode != "local":
        return
        
    if x < 0:
        x = 0
    if y < 0:
        y = 0

    button_name = button.name
    action = 'down' if pressed else 'up'
    
    logger.debug(f"Local mouse click: {button_name} {action} at ({x}, {y})")
    ch9329.mouse.absolute_move(int(x), int(y))
    send_mouse_button(button_name, action)

def on_scroll(x, y, dx, dy):
    if control_mode != "local":
        return
        
    # Update mouse position first
    ch9329.mouse.wheel(dy)
    logger.debug(f"Local mouse scrolled: dy={dy}")

# Keyboard callbacks
def on_press(key):
    if control_mode != "local":
        return
        
    try:
        k = key.char
        logger.debug(f"Local key pressed: {k}")
        handle_key_press(k, "press")
    except AttributeError:
        k = key.name
        if k in KEY_MAPPING:
            logger.debug(f"Local modifier pressed: {k}")
            current_modifier = KEY_MAPPING[k]
        else:
            logger.debug(f"Local special key pressed: {k}")
            handle_key_press(k, "press")

def on_release(key):
    if control_mode != "local":
        return
        
    ch9329.keyboard.release()
    try:
        k = key.char
        logger.debug(f"Local key released: {k}")
    except AttributeError:
        logger.debug(f"Local special key released: {key}")

def handle_key_press(key, action="press"):
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
        ch9329.keyboard.press(k, modifiers=mods)
        logger.info(f"Key combo: Ctrl+{k.upper()}")
    elif key == 'f1':
        ch9329.keyboard.send(("ctrl", "alt", "del", "", "", ""), modifiers=[])
        logger.info("Special: Ctrl+Alt+Del")
    elif key == 'f2':
        ch9329.keyboard.write("906120")
        logger.info("Typed '906120'")
    elif key == 'f3':
        ch9329.keyboard.write("ZYmeng94")
        logger.info("Typed 'ZYmeng94'")
    elif key is None:
        ch9329.keyboard.press("/", modifiers=['ctrl_right'])
    else:
        mapped_key = KEY_MAPPING.get(key, key)
        ch9329.keyboard.press(mapped_key)

def start_local_listeners():
    global mouse_listener, keyboard_listener
    logger.info("Starting local input listeners...")
    
    # Stop existing listeners if running
    stop_local_listeners()
    
    # Create new listeners
    mouse_listener = mouse.Listener(
        on_move=on_move, 
        on_click=on_click,
        on_scroll=on_scroll
    )
    keyboard_listener = keyboard.Listener(
        on_press=on_press, 
        on_release=on_release
    )
    
    # Start listeners
    mouse_listener.start()
    keyboard_listener.start()
    logger.info("Local input listeners started")

def stop_local_listeners():
    global mouse_listener, keyboard_listener
    if mouse_listener:
        mouse_listener.stop()
    if keyboard_listener:
        keyboard_listener.stop()
    mouse_listener = None
    keyboard_listener = None
    logger.info("Local input listeners stopped")

def handle_remote_event(data):
    try:
        event_type = data.get('type')
        if not event_type:
            logger.warning("Received event without type")
            return
            
        if event_type == 'mouse_move':
            x = data.get('x', 0)
            y = data.get('y', 0)
            ch9329.mouse.absolute_move(x, y)
            logger.debug(f"Remote mouse moved to ({x}, {y})")
        elif event_type == 'mouse_click':
            button = data.get('button', 'left')
            action = data.get('action', 'down')
            send_mouse_button(button, action)
            logger.debug(f"Remote mouse click: {button} {action}")
        elif event_type == 'mouse_scroll':
            dy = data.get('dy', 0)
            ch9329.mouse.wheel(dy)
            logger.debug(f"Remote mouse scrolled: dy={dy}")
        elif event_type == 'key_press':
            key = data.get('key')
            if key:
                handle_key_press(key)
                logger.debug(f"Remote key pressed: {key}")
        elif event_type == 'key_release':
            ch9329.keyboard.release()
            logger.debug("Remote key released")
        else:
            logger.warning(f"Unknown event type: {event_type}")
    except Exception as e:
        logger.error(f"Error handling remote event: {str(e)}")

def on_mqtt_message(client, userdata, msg):
    global control_mode
    
    topic = msg.topic
    payload = msg.payload.decode()
    
    if topic == CONTROL_MODE_TOPIC:
        new_mode = payload.lower()
        if new_mode in ["local", "remote"]:
            logger.info(f"Received control mode change: {new_mode}")
            if new_mode != control_mode:
                control_mode = new_mode
                if control_mode == "local":
                    start_local_listeners()
                else:
                    stop_local_listeners()
        else:
            logger.warning(f"Invalid control mode: {new_mode}")
    elif topic == REMOTE_INPUT_TOPIC and control_mode == "remote":
        try:
            data = json.loads(payload)
            handle_remote_event(data)
        except json.JSONDecodeError:
            logger.error("Failed to parse JSON from remote input topic")

def on_mqtt_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(CONTROL_MODE_TOPIC)
        client.subscribe(REMOTE_INPUT_TOPIC)
        logger.info("Connected to MQTT broker and subscribed to topics")
    else:
        reasons = {
            1: "unacceptable protocol version",
            2: "identifier rejected",
            3: "server unavailable",
            4: "bad username or password",
            5: "not authorized"
        }
        reason = reasons.get(rc, f"unknown error (code {rc})")
        logger.error(f"MQTT connection failed: {reason}")

def on_mqtt_disconnect(client, userdata, rc):
    if rc != 0:
        logger.warning(f"Unexpected MQTT disconnection, reconnecting... (rc={rc})")
        client.reconnect()

def load_mqtt_config():
    try:
        with open(MQTT_CONFIG_PATH, "r") as f:
            config = json.load(f)
            # Validate config
            required_keys = ["broker", "port", "username", "password"]
            if all(key in config for key in required_keys):
                return config
            else:
                logger.error("MQTT config missing required keys")
                return None
    except Exception as e:
        logger.error(f"Failed to load MQTT config: {str(e)}")
        return None

def start_mqtt_client():
    global mqtt_client
    
    config = load_mqtt_config()
    if not config:
        logger.error("MQTT client not started due to missing configuration")
        return
        
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_mqtt_connect
    mqtt_client.on_disconnect = on_mqtt_disconnect
    mqtt_client.on_message = on_mqtt_message
    
    try:
        mqtt_client.tls_set(tls_version=ssl.PROTOCOL_TLS)
        mqtt_client.username_pw_set(config["username"], config["password"])
        mqtt_client.connect(config["broker"], config["port"], 60)
        logger.info("Starting MQTT client loop...")
        mqtt_client.loop_start()
    except Exception as e:
        logger.error(f"Failed to start MQTT client: {str(e)}")

def cleanup():
    logger.info("Cleaning up resources...")
    stop_local_listeners()
    if mqtt_client:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
    ch9329.close()

if __name__ == "__main__":
    try:
        logger.info("Starting input controller...")
        logger.info(f"Default control mode: {control_mode}")
        
        # Start with local control
        start_local_listeners()
        
        # Start MQTT client in background
        start_mqtt_client()
        
        logger.info("Controller running. Press Ctrl+C to exit.")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("\nExiting...")
    except Exception as e:
        logger.error(f"Critical error: {str(e)}")
    finally:
        cleanup()