import time
import json
import paho.mqtt.client as mqtt
import paho
import os 

TOPIC = "latency_test"

def load_config():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.abspath(os.path.join(script_dir, "..", ".."))
    config_path = os.path.join(root_dir, "mqtt_config.json")
    with open(config_path, "r") as f:
        return json.load(f)

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Publisher connected to MQTT broker")
    else:
        print(f"Publisher connection failed with error code {rc}")

def on_disconnect(client, userdata, rc):
    print(f"Publisher disconnected (rc={rc})")
    if rc != 0:
        print("Reconnecting publisher...")
        client.reconnect()

def main():
    cfg = load_config()
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    
    client.tls_set(tls_version=paho.mqtt.client.ssl.PROTOCOL_TLS)
    client.username_pw_set(cfg["username"], cfg["password"])
    client.connect(cfg["broker"], cfg["port"], 60)
    client.loop_start()  # Start background thread
    
    message_count = 0
    try:
        while message_count < 20:  # Send 20 test messages
            payload = {
                'timestamp': time.time(),
                'count': message_count
            }
            client.publish(TOPIC, json.dumps(payload))
            print(f"Published message {message_count}")
            message_count += 1
            time.sleep(1)  # Send every second
            
    except KeyboardInterrupt:
        print("Publisher stopped")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()