import sounddevice as sd
import websockets
import asyncio
import numpy as np

RECORD_SECONDS = 5
SAMPLE_RATE = 16000
CHANNELS = 2
CHUNK_SIZE = 1024  # Adjust as needed

print("* recording")

# Record audio for 5 seconds
audio_data = sd.rec(int(RECORD_SECONDS * SAMPLE_RATE), samplerate=SAMPLE_RATE, channels=CHANNELS)
sd.wait()  # Wait until recording is finished
audio_data = audio_data.flatten().tolist()  # Convert to list for sending

print("* done recording")

async def client():
    async with websockets.connect("ws://localhost:8765") as websocket:
        for i in range(0, len(audio_data), CHUNK_SIZE):
            chunk = audio_data[i:i+CHUNK_SIZE]
            await websocket.send(np.array(chunk).tobytes())

asyncio.get_event_loop().run_until_complete(client())



# websocket.onopen = function() {
#       console.log("Connected to server.");
      
#       websocket.send(JSON.stringify({
#         uid: generateUUID(),
#         multilingual: false,
#         language: "en",
#         task: "transcribe"
#       }));
#     }
