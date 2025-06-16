import json
import paho.mqtt.client as mqtt
import paho
import os 

Topic = "alert"

def load_config():
    # Get current script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up two levels to reach armController directory
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    config_path = os.path.join(root_dir, "mqtt_config.json")
    with open(config_path, "r") as f:
        return json.load(f)

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    client.subscribe(Topic)

def on_message(client, userdata, msg):  
    print(f"Received message on topic '{msg.topic}': {msg.payload.decode()}")

def main():
    cfg = load_config()

    client = mqtt.Client(userdata=cfg)
    client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
    client.username_pw_set(cfg["username"], cfg["password"])
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(cfg["broker"], cfg["port"], 60)

    client.loop_forever()

if __name__ == "__main__":
    main()
