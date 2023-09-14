# import websocket
# import json

# def on_message(ws, message):
#     print(message)


# def on_open(ws):
#     start = {
#     "action": "Enable",
#     }
#     ws.send(json.dumps(start))


# ws_url = "ws://localhost:51512/api/WebSocket"
# # create a WebSocket object and set the callback function
# # start the WebSocket connection
# ws = websocket.WebSocketApp(ws_url, on_message=on_message) #, on_open=on_open
# ws.run_forever()

import websockets
import asyncio

# The main function that will handle connection and communication 
# with the server
async def listen():
    url = "ws://127.0.0.1:7890/arm"
    # Connect to the server
    async with websockets.connect(url) as ws:
        # Send a greeting message
        await ws.send("Hello Server!")
        # Stay alive forever, listening to incoming msgs
        while True:
            try:
                message = await ws.recv()
                print(f"Received message: {message}")
            except websockets.exceptions.ConnectionClosedError:
                print("Connection to the server closed.")
                break

# Start the connection
asyncio.get_event_loop().run_until_complete(listen())
