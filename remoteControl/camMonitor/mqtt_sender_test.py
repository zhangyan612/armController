import json
import time
import paho.mqtt.client as mqtt
import paho
import os


Topic = "alert"

# def load_config():
#     with open("mqtt_config.json", "r") as f:
#         return json.load(f)
    
def load_config():
    # Get current script directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Go up two levels to reach armController directory
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    config_path = os.path.join(root_dir, "mqtt_config.json")
    with open(config_path, "r") as f:
        return json.load(f)


def main():
    cfg = load_config()

    client = mqtt.Client()
    client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
    client.username_pw_set(cfg["username"], cfg["password"])
    client.connect(cfg["broker"], cfg["port"], 60)
    client.loop_start()

    print(f"Publishing test message to topic '{Topic}'...")
    client.publish(Topic, "alert2")
    time.sleep(1)
    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    main()
