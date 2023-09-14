import asyncio
import websockets

# Dictionary to store all connected clients for each path
clients = {}

# Function to broadcast a message to all clients connected to a specific path
async def broadcast(path, message):
    if path in clients:
        await asyncio.wait([client.send(message) for client in clients[path]])

# WebSocket server handler
async def server(websocket, path):
    # Extract the path from the URL (e.g., /chatroom1)
    path = path.lstrip('/')
    
    # Add the new client to the clients dictionary for the specific path
    if path not in clients:
        clients[path] = set()
    clients[path].add(websocket)
    
    try:
        async for message in websocket:
            # Broadcast the received message to all clients on the same path
            await broadcast(path, message)
    except websockets.exceptions.ConnectionClosedError:
        pass
    finally:
        # Remove the client from the clients set when they disconnect
        clients[path].remove(websocket)

# Start the WebSocket server
start_server = websockets.serve(server, "localhost", 8765)

async def main():
    await start_server
    await asyncio.Future()  # Run indefinitely

if __name__ == "__main__":
    asyncio.run(main())
