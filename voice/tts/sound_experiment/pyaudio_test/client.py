import pyaudio
import asyncio
import websockets


class micRecorder:
    def __init__(self) -> None:
        self.CHUNK = 1024 * 3
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.p = pyaudio.PyAudio()

        self.stream = self.p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)

    def record_microphone(self, stream):
        CHUNK = 1024 *3
        while True:
            data = stream.read(CHUNK)
            yield data

    async def send_audio(self):
        async with websockets.connect('ws://localhost:8888') as ws:
            for data in self.record_microphone(self.stream):

                await ws.send(data)


recorder = micRecorder()
asyncio.get_event_loop().run_until_complete(recorder.send_audio())
