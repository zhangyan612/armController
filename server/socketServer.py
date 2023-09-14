import websockets
import asyncio

# Server data
PORT = 7890
print("Server listening on Port " + str(PORT))

# A set of connected ws clients
connected = {}

# The main behavior function for this server
async def echo(websocket, path):
    print("A client just connected to path: " + path)
    # Store the connected client based on their path
    connected[path] = websocket
    
    # Handle incoming messages
    try:
        async for message in websocket:
            print("Received message from client: " + message)
            # Send a response to all connected clients except sender
            if path == "/arm":
                for path, conn in connected.items():
                    if path == "/arm":
                        url = "ws://localhost:8080/api/ws"
                        # Connect to the server
                        async with websockets.connect(url) as ws:
                            # Stay alive forever, listening to incoming msgs
                            while True:
                                try:
                                    message = await ws.recv()
                                    if message:
                                        await conn.send(message)
                                except websockets.exceptions.ConnectionClosedError:
                                    print("Connection to the server closed.")
                                    break

    # Handle disconnecting clients 
    except websockets.exceptions.ConnectionClosed as e:
        print("A client just disconnected from path: " + path)
    finally:
        del connected[path]

# Start the server
start_server = websockets.serve(echo, "localhost", PORT)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
