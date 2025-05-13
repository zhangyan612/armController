#！/usr/bin/env python3
import json
import paho.mqtt.client as mqtt
import pyautogui

# pip install paho-mqtt
# pip install pyautogui  accept icon location 1602, 967

MQTT_BROKER = "broker.mqttdashboard.com"
MQTT_PORT = 1883
TOPIC = "yzhang/mouse_keyboard"

def on_message(client, userdata, msg):
    try:
        event = json.loads(msg.payload.decode())
        etype = event.get('type')

        if etype == 'move':
            print(f"Mouse moved to: {event['x']}, {event['y']}")
            # pyautogui.moveTo(event['x'], event['y'], duration=0)

        elif etype == 'mouse':
            pyautogui.moveTo(event['x'], event['y'])
            if event['action'] == 'down':
                print(f"Mouse down: {event['button']}")
                # pyautogui.mouseDown(button=event['button'])
            else:
                print(f"Mouse up: {event['button']}")
                # pyautogui.mouseUp(button=event['button'])

        elif etype == 'keyboard':
            key = event['key']
            if event['action'] == 'press':
                print(f"Key Down: {key}")
                # pyautogui.keyDown(key)
            else:
                print(f"Key up: {key}")
                # pyautogui.keyUp(key)

    except Exception as e:
        print("Error processing message:", e)

client = mqtt.Client()
client.on_message = on_message
client.connect(MQTT_BROKER, MQTT_PORT, 60)
client.subscribe(TOPIC)

client.loop_forever()
