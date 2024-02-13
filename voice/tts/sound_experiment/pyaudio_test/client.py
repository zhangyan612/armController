import pyaudio
import asyncio
import websockets

CHUNK = 1024 * 3
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

def record_microphone(stream):
    CHUNK = 1024 *3
    while True:
        data = stream.read(CHUNK)
        yield data

async def send_audio():
    async with websockets.connect('ws://localhost:8888') as ws:

        for data in record_microphone(stream):
            await ws.send(data)

asyncio.get_event_loop().run_until_complete(send_audio())
