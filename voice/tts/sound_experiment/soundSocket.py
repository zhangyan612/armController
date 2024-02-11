import sounddevice as sd
import numpy as np
import asyncio
import websockets
import json
import uuid

# Buffer size
chunk_size = 4096

# Buffer
buffer = np.zeros(chunk_size, dtype=np.float32)

# Buffer pointer
buffer_pointer = 0

# Websocket state
websocket_state = 0

# Server state
server_state = 1

# Websocket URI
websocket_uri = "ws://localhost:6006"  # replace with your WebSocket URI

async def start_recording(websocket):
    global buffer, buffer_pointer, server_state, websocket_state

    def audio_callback(indata, frames, time, status):
        global buffer, buffer_pointer, server_state, websocket_state

        # Process audio input
        for sample in indata:
            buffer[buffer_pointer] = sample[0]
            buffer_pointer += 1

            # If buffer is full
            if buffer_pointer >= chunk_size:
                # If server is ready and websocket is open
                if server_state == 1 and websocket_state == 0:
                    # Send buffer
                    asyncio.run(websocket.send(buffer.tobytes()))
                    print("send data")

                # Reset buffer pointer
                buffer_pointer = 0

    # Create stream
    stream = sd.InputStream(callback=audio_callback)
    with stream:
        while True:
            await asyncio.sleep(0.1)

async def start_websocket():
    global websocket_state

    async with websockets.connect(websocket_uri) as websocket:
        print("Websocket created.")

        await websocket.send(json.dumps({
            "uid": str(uuid.uuid4()),
            "multilingual": False,
            "language": "en",
            "task": "transcribe"
        }))

        print("Connected to server.")
        await start_recording(websocket)

# Run the WebSocket
asyncio.get_event_loop().run_until_complete(start_websocket())
# asyncio.get_event_loop().run_forever()
