import asyncio
import websockets
import time
from whisperspeech.pipeline import Pipeline


async def tts_test(websocket, path):
    # Wait for UI to connect
    await websocket.wait_closed()
    print("UI connected.")

    # Initialize the pipeline
    pipe = Pipeline(s2a_ref='collabora/whisperspeech:s2a-q4-tiny-en+pl.model', torch_compile=True)

    # Generate audio
    text = 'this is test run'
    audio = pipe.generate(text)
    output_audio = audio.cpu().numpy()
    bytesData = output_audio.tobytes()
    websocket.ping()

    # Send audio data to UI
    await websocket.send(bytesData)
    print("Audio data sent to UI.")

# Start the WebSocket server
start_server = websockets.serve(tts_test, "0.0.0.0", 8888)

# Run the server
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
