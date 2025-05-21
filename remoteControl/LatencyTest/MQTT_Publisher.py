import time
import json
import paho.mqtt.client as mqtt

MQTT_BROKER = "broker.mqttdashboard.com"
TOPIC = "yzhang/latency"

client = mqtt.Client()
client.connect(MQTT_BROKER, 1883, 60)

for _ in range(10):
    payload = {"timestamp": time.time()}
    client.publish(TOPIC, json.dumps(payload))
    time.sleep(0.2)
