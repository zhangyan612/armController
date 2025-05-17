import json
import oci
import time
import pyautogui
from base64 import b64decode

ociMessageEndpoint = "https://cell-1.streaming.us-chicago-1.oci.oraclecloud.com"  
ociStreamOcid = "ocid1.stream.oc1.us-chicago-1.amaaaaaaoa2s45aam2sr7zfgu34z365rgfkeuch5qule2b2dbbpztvoudq6q"  
ociConfigFilePath = "C:\\Users\\yanzh\\.oci\\config"
ociProfileName = "DEFAULT"  


def get_cursor_by_group(sc, sid, group_name, instance_name):
    cursor_details = oci.streaming.models.CreateGroupCursorDetails(
        group_name=group_name,
        instance_name=instance_name,
        type=oci.streaming.models.CreateGroupCursorDetails.TYPE_LATEST,  # Only get new messages
        commit_on_get=True
    )
    response = sc.create_group_cursor(sid, cursor_details)
    return response.data.value

def simple_message_loop(client, stream_id, initial_cursor):
    cursor = initial_cursor
    while True:
        get_response = client.get_messages(stream_id, cursor, limit=10)
        for message in get_response.data:
            try:
                decoded = b64decode(message.value.encode()).decode()
                event = json.loads(decoded)
                handle_event(event)
            except Exception as e:
                print("Error processing message:", e)
        cursor = get_response.headers["opc-next-cursor"]
        time.sleep(0.1)

def handle_event(event):
    etype = event.get('type')
    if etype == 'move':
        print(f"Mouse moved to: {event['x']}, {event['y']}")
        pyautogui.moveTo(event['x'], event['y'], duration=0)
    elif etype == 'mouse':
        print(f"Mouse moved to: {event['x']}, {event['y']}")
        pyautogui.moveTo(event['x'], event['y'])
        if event['action'] == 'down':
            print(f"Mouse {event['button']} down at: {event['x']}, {event['y']}")
            pyautogui.mouseDown(button=event['button'])
        else:
            pyautogui.mouseUp(button=event['button'])
    elif etype == 'keyboard':
        key = event['key']
        if event['action'] == 'press':
            pyautogui.keyDown(key)
        else:
            pyautogui.keyUp(key)

# Setup OCI client
config = oci.config.from_file(ociConfigFilePath, ociProfileName)
stream_client = oci.streaming.StreamClient(config, service_endpoint=ociMessageEndpoint)
group_cursor = get_cursor_by_group(stream_client, ociStreamOcid, "mouse-group", "instance-1")
simple_message_loop(stream_client, ociStreamOcid, group_cursor)
