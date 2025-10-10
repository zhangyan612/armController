import time
import json
import paho.mqtt.client as mqtt
import paho
import os 

TOPIC = "joint_state"

def load_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, ".."))
    config_path = os.path.join(root_dir, "mqtt_config.json")
    with open(config_path, "r") as f:
        return json.load(f)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Receiver connected to MQTT broker")
        client.subscribe(TOPIC)
    else:
        print(f"Receiver connection failed with error code {rc}")

def on_disconnect(client, userdata, rc):
    print(f"Receiver disconnected (rc={rc})")
    if rc != 0:
        print("Reconnecting receiver...")
        client.reconnect()

def on_message(client, userdata, msg):
    try:
        # Parse the raw list of potentiometer readings
        pot_readings = json.loads(msg.payload.decode())
        print(f"Received potentiometer readings: {pot_readings}")
        
    except Exception as e:
        print(f"Error processing message: {str(e)}")

def main():
    cfg = load_config()
    
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    
    client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
    client.username_pw_set(cfg["username"], cfg["password"])
    client.connect(cfg["broker"], cfg["port"], 60)
    
    print(f"Subscribed to topic: {TOPIC}")
    print("Waiting for potentiometer readings...")
    
    client.loop_forever()

if __name__ == "__main__":
    main()