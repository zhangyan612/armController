import time
import json
from pynput import mouse, keyboard
import math
import paho.mqtt.client as mqtt
import os
import logging
import ssl

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('input_publisher')

# MQTT Configuration
TOPIC = "remote_keyboard_mouse"
MQTT_CONFIG_PATH = "mqtt_config.json"

# Position cache and thresholds
current_pos = {'x': 0, 'y': 0}
last_sent_pos = {'x': 0, 'y': 0}
MOVE_THRESHOLD = 10

def distance(p1, p2):
    return math.hypot(p1['x'] - p2['x'], p1['y'] - p2['y'])

def load_config():
    try:
        with open(MQTT_CONFIG_PATH, "r") as f:
            return json.load(f)
    except Exception as e:
        logger.error(f"Failed to load config: {str(e)}")
        raise

def setup_mqtt(cfg):
    client = mqtt.Client()
    
    # Connection callbacks
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            logger.info("Connected to MQTT broker")
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
    
    # Add TLS configuration if using secure connection
    if cfg.get("tls", False) or cfg["port"] == 8883:
        logger.info("Configuring TLS connection")
        client.tls_set(tls_version=ssl.PROTOCOL_TLS)
        client.tls_insecure_set(True)  # For testing only
    
    # Set credentials if provided
    if "username" in cfg and "password" in cfg:
        client.username_pw_set(cfg["username"], cfg["password"])
    
    client.connect(cfg["broker"], cfg["port"], 60)
    client.loop_start()
    return client

# Mouse event handlers
def on_move(x, y):
    global current_pos, last_sent_pos
    current_pos = {'x': int(x), 'y': int(y)}
    
    if distance(current_pos, last_sent_pos) >= MOVE_THRESHOLD:
        payload = {
            'type': 'mouse_move',
            'x': current_pos['x'],
            'y': current_pos['y'],
            'timestamp': time.time()
        }
        logger.info(f"Publishing mouse move: ({current_pos['x']}, {current_pos['y']})")
        mqtt_client.publish(TOPIC, json.dumps(payload))
        last_sent_pos = current_pos.copy()

def on_click(x, y, button, pressed):
    action = 'down' if pressed else 'up'
    payload = {
        'type': 'mouse_click',
        'x': int(x),
        'y': int(y),
        'button': button.name,
        'action': action,
        'timestamp': time.time()
    }
    logger.info(f"Publishing mouse click: {button.name} {action} at ({x}, {y})")
    mqtt_client.publish(TOPIC, json.dumps(payload))

def on_scroll(x, y, dx, dy):
    payload = {
        'type': 'mouse_scroll',
        'x': int(x),
        'y': int(y),
        'dx': int(dx),
        'dy': int(dy),
        'timestamp': time.time()
    }
    logger.info(f"Publishing mouse scroll: dx={dx}, dy={dy} at ({x}, {y})")
    mqtt_client.publish(TOPIC, json.dumps(payload))

# Keyboard event handlers
def on_press(key):
    try:
        k = key.char
        payload = {
            'type': 'key_press',
            'key': k,
            'action': 'press',
            'timestamp': time.time()
        }
        logger.info(f"Publishing key press: {k}")
    except AttributeError:
        k = key.name
        payload = {
            'type': 'key_press',
            'key': k,
            'action': 'press',
            'timestamp': time.time()
        }
        logger.info(f"Publishing special key press: {k}")
    
    mqtt_client.publish(TOPIC, json.dumps(payload))

def on_release(key):
    try:
        k = key.char
    except AttributeError:
        k = key.name
        
    payload = {
        'type': 'key_release',
        'key': k,
        'action': 'release',
        'timestamp': time.time()
    }
    logger.info(f"Publishing key release: {k}")
    mqtt_client.publish(TOPIC, json.dumps(payload))

if __name__ == "__main__":
    try:
        # MQTT Setup
        logger.info("Starting input publisher...")
        cfg = load_config()
        mqtt_client = setup_mqtt(cfg)
        time.sleep(1)  # Allow connection time
        
        # Start listeners
        mouse_listener = mouse.Listener(
            on_move=on_move, 
            on_click=on_click,
            on_scroll=on_scroll
        )
        keyboard_listener = keyboard.Listener(
            on_press=on_press, 
            on_release=on_release
        )
        
        mouse_listener.start()
        keyboard_listener.start()
        
        logger.info("Input publisher started. Capturing events...")
        
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\nStopping publisher...")
        mouse_listener.stop()
        keyboard_listener.stop()
        mqtt_client.loop_stop()
    except Exception as e:
        logger.error(f"Critical error: {str(e)}")