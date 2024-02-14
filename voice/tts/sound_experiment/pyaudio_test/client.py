import pyaudio
import asyncio
import websockets
import torch
import time
# from vad import VoiceActivityDetection
import numpy as np
import webrtcvad

class micRecorder:
    def __init__(self) -> None:
        self.CHUNK = 1024 * 3
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.p = pyaudio.PyAudio()
        self.vad_threshold = 0.5

        self.stream = self.p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)
        self.clients = {}
        self.eos = False
        self.webrtc_vad_model = webrtcvad.Vad()
        self.webrtc_vad_model.set_mode(1)  # set aggressiveness mode, in [0, 3]

    def _is_webrtc_speech(self, data, all_frames_must_be_true=False):
        """
        Returns true if speech is detected in the provided audio data

        Args:
            data (bytes): raw bytes of audio data (1024 raw bytes with
            16000 sample rate and 16 bits per sample)
        """
        sample_rate = self.RATE
        # Number of audio frames per millisecond
        frame_length = int(sample_rate * 0.01)  # for 10ms frame
        num_frames = int(len(data) / (2 * frame_length))
        speech_frames = 0

        for i in range(num_frames):
            start_byte = i * frame_length * 2
            end_byte = start_byte + frame_length * 2
            frame = data[start_byte:end_byte]
            if self.webrtc_vad_model.is_speech(frame, sample_rate):
                speech_frames += 1
                if not all_frames_must_be_true:
                    return True
        if all_frames_must_be_true:
            return speech_frames == num_frames
        else:
            return False


    def record_microphone(self, stream):
        while True:
            data = stream.read(self.CHUNK)
            yield data

    def recording(self):
        for data in self.record_microphone(self.stream):
            voice_detected = self._is_webrtc_speech(data)
            if voice_detected:
                print("Voice detected!")
            else:
                print("No voice detected.")

    def bytes_to_float_array(self, audio_bytes):
        raw_data = np.frombuffer(buffer=audio_bytes, dtype=np.int16)
        return raw_data.astype(np.float32) / 32768.0

    async def send_audio(self):
        async with websockets.connect('ws://localhost:6006') as ws:
            for data in self.record_microphone(self.stream):
                voice_detected = self._is_webrtc_speech(data)
                if voice_detected:
                    print('Voice detected, send data')
                    audio_array = self.bytes_to_float_array(data)
                    await ws.send(audio_array.tobytes())
                    # self.send_packet_to_server(audio_array.tobytes())
                    # await ws.send(data)
                else:
                    print("No voice detected.")



recorder = micRecorder()
# recorder.recording()
asyncio.get_event_loop().run_until_complete(recorder.send_audio())
