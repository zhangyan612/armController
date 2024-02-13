import pyaudio
import asyncio
import websockets
import torch
import time
from vad import VoiceActivityDetection
import numpy as np

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
        self.clients = {}
        self.eos = False

    def voice_activity_detection(self, frame_np, no_voice_activity_chunks):
        try:
            speech_prob = self.vad_model(torch.from_numpy(frame_np.copy()), self.RATE).item()
            if speech_prob < self.vad_threshold:
                no_voice_activity_chunks += 1
                if no_voice_activity_chunks > 3:
                    if not self.eos:
                        self.eos = True
                    time.sleep(0.1)
                return no_voice_activity_chunks, False
            no_voice_activity_chunks = 0
            self.eos = False
            return no_voice_activity_chunks, True
        
        except Exception as e:
            print(e)
            return no_voice_activity_chunks, False

    def record_microphone(self, stream):
        while True:
            data = stream.read(self.CHUNK)
            yield data

    async def send_audio(self):
        self.vad_model = VoiceActivityDetection()
        self.vad_threshold = 0.5
        no_voice_activity_chunks = 0

        async with websockets.connect('ws://localhost:6006') as ws:
            for data in self.record_microphone(self.stream):
                frame_np = np.frombuffer(data, dtype=np.float32)
                no_voice_activity_chunks, continue_processing = self.voice_activity_detection(frame_np, no_voice_activity_chunks)
                if not continue_processing:
                    print('skip')
                    continue
                await ws.send(data)


recorder = micRecorder()
asyncio.get_event_loop().run_until_complete(recorder.send_audio())
