import time
import json
import paho.mqtt.client as mqtt
from serial import Serial
from ch9329 import Keyboard, Mouse
import os
import logging
import ssl
import paho

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('input_receiver')

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

# Global controller instance
controller = None

class CH9329Controller:
    def __init__(self, port="COM12", baudrate=9600, screenx=1920, screeny=1080):
        try:
            self.ser = Serial(port, baudrate, timeout=1)
            self.keyboard = Keyboard(self.ser)
            self.mouse = Mouse(self.ser, screenx, screeny)
            self.current_pos = {'x': 0, 'y': 0}
            logger.info(f"CH9329 initialized on {port}")
        except Exception as e:
            logger.error(f"Failed to initialize CH9329: {str(e)}")
            raise
        
    def handle_mouse_move(self, data):
        try:
            self.mouse.absolute_move(data['x'], data['y'])
            self.current_pos = {'x': data['x'], 'y': data['y']}
            logger.debug(f"Mouse moved to ({data['x']}, {data['y']})")
        except Exception as e:
            logger.error(f"Mouse move error: {str(e)}")
    
    def handle_mouse_click(self, data):
        try:
            button = data['button']
            action = data['action']
            
            if button == 'left':
                if action == 'down': 
                    self.mouse.press("left")
                    logger.debug("Left mouse button pressed")
                else: 
                    self.mouse.release()
                    logger.debug("Mouse button released")
            elif button == 'right':
                if action == 'down': 
                    self.mouse.press("right")
                    logger.debug("Right mouse button pressed")
                else: 
                    self.mouse.release()
                    logger.debug("Mouse button released")
            elif button == 'middle':
                if action == 'down': 
                    self.mouse.press("center")
                    logger.debug("Middle mouse button pressed")
                else: 
                    self.mouse.release()
                    logger.debug("Mouse button released")
        except Exception as e:
            logger.error(f"Mouse click error: {str(e)}")
    
    def handle_mouse_scroll(self, data):
        try:
            self.mouse.wheel(data['dy'])
            logger.debug(f"Mouse scrolled: dy={data['dy']}")
        except Exception as e:
            logger.error(f"Mouse scroll error: {str(e)}")
    
    def handle_key_press(self, data):
        try:
            key = data['key']
            latency_ms = (time.time() - data['timestamp']) * 1000
            
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
                logger.info(f"Key combo: Ctrl+{k.upper()} (Latency: {latency_ms:.2f}ms)")
            elif key == 'f1':
                self.keyboard.send(("ctrl", "alt", "del", "", "", ""), modifiers=[])
                logger.info(f"Special: Ctrl+Alt+Del (Latency: {latency_ms:.2f}ms)")
            elif key == 'f2':
                self.keyboard.write("906120")
                logger.info(f"Typed '906120' (Latency: {latency_ms:.2f}ms)")
            elif key == 'f3':
                self.keyboard.write("ZYmeng94")
                logger.info(f"Typed 'ZYmeng94' (Latency: {latency_ms:.2f}ms)")
            else:
                # Map special keys
                mapped_key = KEY_MAPPING.get(key, key)
                self.keyboard.press(mapped_key)
                logger.info(f"Key pressed: {mapped_key} (Latency: {latency_ms:.2f}ms)")
        except Exception as e:
            logger.error(f"Key press error: {str(e)}")
    
    def handle_key_release(self, data):
        try:
            latency_ms = (time.time() - data['timestamp']) * 1000
            self.keyboard.release()
            logger.info(f"Key released: {data['key']} (Latency: {latency_ms:.2f}ms)")
        except Exception as e:
            logger.error(f"Key release error: {str(e)}")

def on_message(client, userdata, msg):
    global controller
    try:
        data = json.loads(msg.payload.decode())
        event_type = data['type']
        
        # Calculate latency
        current_time = time.time()
        latency_ms = (current_time - data['timestamp']) * 1000
        
        logger.info(f"Received {event_type} event (Latency: {latency_ms:.2f}ms)")
        
        if controller is None:
            logger.error("Controller not initialized - cannot process event")
            return
            
        if event_type == 'mouse_move':
            controller.handle_mouse_move(data)
        elif event_type == 'mouse_click':
            controller.handle_mouse_click(data)
        elif event_type == 'mouse_scroll':
            controller.handle_mouse_scroll(data)
        elif event_type == 'key_press':
            controller.handle_key_press(data)
        elif event_type == 'key_release':
            controller.handle_key_release(data)
            
    except Exception as e:
        logger.error(f"Message handling error: {str(e)}")

def load_config():
    try:
        with open(MQTT_CONFIG_PATH, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Failed to load config: {str(e)}")
        raise

if __name__ == "__main__":
    # global controller
    try:
        logger.info("Starting input receiver...")
        
        # Setup CH9329 controller
        controller = CH9329Controller()
        
        # MQTT Setup
        cfg = load_config()
        client = mqtt.Client()
        client.on_message = on_message
        
        # Connection callbacks
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                client.subscribe(TOPIC)
                logger.info("Connected to MQTT broker and subscribed")
            else:
                reasons = {
                    1: "unacceptable protocol version",
                    2: "identifier rejected",
                    3: "server unavailable",
                    4: "bad username or password",
                    5: "not authorized"
                }
                reason = reasons.get(rc, f"unknown error (code {rc})")
                logger.error(f"Connection failed: {reason}")
                
        def on_disconnect(client, userdata, rc):
            if rc != 0:
                reasons = {
                    1: "unacceptable protocol version",
                    2: "identifier rejected",
                    3: "server unavailable",
                    4: "bad username or password",
                    5: "not authorized"
                }
                reason = reasons.get(rc, f"unknown error (code {rc})")
                logger.warning(f"Unexpected disconnection: {reason}, reconnecting...")
                client.reconnect()
                
        client.on_connect = on_connect
        client.on_disconnect = on_disconnect
        
        client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
        client.username_pw_set(cfg["username"], cfg["password"])
        
        client.connect(cfg["broker"], cfg["port"], 60)
        
        logger.info("Input receiver started. Listening for events...")
        client.loop_forever()
    except KeyboardInterrupt:
        logger.info("\nReceiver stopped by user")
        # Clean up serial connection
        if controller and controller.ser and controller.ser.is_open:
            controller.ser.close()
    except Exception as e:
        logger.error(f"Critical error: {str(e)}")
        if controller and controller.ser and controller.ser.is_open:
            controller.ser.close()