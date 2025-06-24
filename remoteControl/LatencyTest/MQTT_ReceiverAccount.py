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



# Message 1 - Latency: 366.83 ms
# Message 2 - Latency: 282.05 ms
# Message 3 - Latency: 284.08 ms
# Message 4 - Latency: 291.47 ms
# Message 5 - Latency: 361.21 ms
# Message 6 - Latency: 373.36 ms
# Message 7 - Latency: 282.35 ms
# Message 8 - Latency: 280.04 ms
# Message 9 - Latency: 278.30 ms
# Message 10 - Latency: 448.88 ms
# Message 11 - Latency: 337.81 ms
# Message 12 - Latency: 345.00 ms
# Message 13 - Latency: 279.74 ms
# Message 14 - Latency: 364.01 ms
# Message 15 - Latency: 279.26 ms
# Message 16 - Latency: 286.88 ms
# Message 17 - Latency: 300.00 ms
# Message 18 - Latency: 314.48 ms
# Message 19 - Latency: 333.94 ms