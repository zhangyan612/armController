import time
import json
import paho.mqtt.client as mqtt

def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    latency = (time.time() - data['timestamp']) * 1000
    print(f"MQTT Latency: {latency:.2f} ms")

client = mqtt.Client()
client.on_message = on_message
client.connect("broker.mqttdashboard.com", 1883, 60)
client.subscribe("yzhang/latency")
client.loop_forever()
