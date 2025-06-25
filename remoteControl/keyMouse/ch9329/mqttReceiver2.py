import time
import json
import paho.mqtt.client as mqtt
import paho
import os 

TOPIC = "latency_test"
MQTT_CONFIG_PATH = "mqtt_config.json"

def load_config():
    try:
        with open(MQTT_CONFIG_PATH, "r") as f:
            return json.load(f)
    except Exception as e:
        print(f"Failed to load config: {str(e)}")
        raise

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
        print('got message')
        data = json.loads(msg.payload.decode())
        receive_time = time.time()
        latency = (receive_time - data['timestamp']) * 1000  # Convert to ms
        print(f"Message {data['count']} - Latency: {latency:.2f} ms")
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
    
    client.loop_forever()

if __name__ == "__main__":
    main()

