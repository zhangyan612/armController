import asyncio
import websockets

async def client():
    uri = "ws://localhost:8765/chatroom1"  # Replace with the desired path

    async with websockets.connect(uri) as websocket:
        while True:
            try:
                message = await websocket.recv()
                print(f"Received message: {message}")
            except websockets.exceptions.ConnectionClosedError:
                print("Connection to the server closed.")
                break

if __name__ == "__main__":
    asyncio.run(client())
