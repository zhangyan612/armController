import os
import json
import paho.mqtt.client as mqtt
from playsound import playsound
import paho

# pip install paho-mqtt playsound==1.2.2

TOPIC = "alert"
# ALARM_FILE = "alarm.wav"  # Ensure this file exists

# Resolve absolute path to alarm.wav in project root
script_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
ALARM_FILE = os.path.join(root_dir, "alarm.wav")

def load_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    config_path = os.path.join(root_dir, "mqtt_config.json")
    with open(config_path, "r") as f:
        return json.load(f)

def play_alarm():
    try:
        print("Playing alarm...")
        playsound(ALARM_FILE)
    except Exception as e:
        print(f"Error playing alarm: {e}")

def on_connect(client, userdata, flags, rc):
    print("Connected to MQTT broker with result code", rc)
    client.subscribe(TOPIC)

def on_message(client, userdata, msg):
    payload = msg.payload.decode()
    print(f"[MQTT] Received message on {msg.topic}: {payload}")
    if payload == "ALERT":
        play_alarm()

def main():
    cfg = load_config()

    client = mqtt.Client()
    client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
    client.username_pw_set(cfg["username"], cfg["password"])
    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(cfg["broker"], cfg["port"], 60)
    client.loop_forever()

if __name__ == "__main__":
    main()
    # playsound(ALARM_FILE)