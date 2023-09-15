import websocket
import json
import portConnect


# when receiving message from server, send it to MQ

def on_message(ws, message):
    msg = json.loads(message)
    connected = msg['payload']['connected']
    print(connected)
    if not connected:
        # http call
        portConnect.connect_robot()

    # if connected:
    #     stop_arm(ws)

    # print(msg['payload']['jointState'])
    if msg['event'] == 'TaskUpdate' and msg['payload']['task'] != None:
        print("Task运行进度:", msg['payload']['progress'] * 100, " %")
    if msg['event'] == 'TaskUpdate' and msg['payload']['task'] == None:
        print("Task运行完成")

def start_arm(ws):
    start = {
    "action": "Enable",
    }
    ws.send(json.dumps(start))

def stop_arm(ws):
    start = {
    "action": "Disable",
    }
    ws.send(json.dumps(start))

def on_open(ws):
    wscript = {
        'action': 'SetTask',
        'payload': {
            'type': 'WScriptTask',
            'args': """
            // WScript goes here
            MOVEL -0.000,0.157,0.204,180,0,-90,3
            """
        }
    }
    start = {
    "action": "Enable",
    }
    disable = {
    "action": "Disable",
    }

    ws.send(json.dumps(start))

def continuousConnection():
    # define the WebSocket URL
    ws_url = "ws://localhost:8080/api/ws"
    # create a WebSocket object and set the callback function
    # start the WebSocket connection
    ws = websocket.WebSocketApp(ws_url, on_message=on_message) #, on_open=on_open
    ws.run_forever()