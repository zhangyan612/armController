# import pyaudio
import numpy as np
import asyncio
import websockets
import threading
import wave 
from transcriber import WhisperModel
import time
import os
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

class TranscriptionServer:
    def __init__(self) -> None:
        # self.CHUNK = 1024 * 3
        self.frames = []
        self.n_audio_file = 0
        self.frames_np = None
        self.frames_offset = 0.0
        self.RATE = 16000
        self.timestamp_offset = 0.0
        self.frames_offset = 0.0
        device = "cpu"
        compute = "int8"
        self.sleep_duration = 0.4

        self.transcriber = WhisperModel(
            model_size_or_path="base.en",
            device=device,
            compute_type=compute,
            local_files_only=False,
        )

    def write_audio_frames_to_file(self, frames, file_name, rate=None):
        with wave.open(file_name, "wb") as wavfile:
            rate = 16000
            wavfile: wave.Wave_write
            wavfile.setnchannels(1)
            wavfile.setsampwidth(2)
            wavfile.setframerate(rate)
            wavfile.writeframes(frames)

    def add_frames(self, frame_np):
        '''
            If the buffer size exceeds a threshold (45 seconds of audio data), it discards the oldest 30 seconds
        '''
        if self.frames_np is not None and self.frames_np.shape[0] > 45*self.RATE:
            self.frames_offset += 30.0
            self.frames_np = self.frames_np[int(30*self.RATE):]
        if self.frames_np is None:
            self.frames_np = frame_np.copy()
        else:
            self.frames_np = np.concatenate((self.frames_np, frame_np), axis=0)

    async def websocket_handler(self, websocket, path):
        try:
            async for message in websocket:
                # print(message)
                # self.frames += message
                frame_np = np.frombuffer(message, dtype=np.float32)
                self.add_frames(frame_np)
                # audio_data = np.frombuffer(message, dtype=np.int16)
                # stream.write(audio_data.tobytes())
                #stream.write(message)
                # if len(self.frames) > 50000: # * self.rate:
                # save every 10 seconds

                # clip audio if the current chunk exceeds 30 seconds, this basically implies that
                # no valid segment for the last 30 seconds from whisper
                if self.frames_np[int((self.timestamp_offset - self.frames_offset)*self.RATE):].shape[0] > 25 * self.RATE:
                    duration = self.frames_np.shape[0] / self.RATE
                    self.timestamp_offset = self.frames_offset + duration - 5

                samples_take = max(0, (self.timestamp_offset - self.frames_offset)*self.RATE)
                input_bytes = self.frames_np[int(samples_take):].copy()
                duration = input_bytes.shape[0] / self.RATE

                print(duration)
                if duration < self.sleep_duration:
                    time.sleep(0.01)    # 5ms sleep to wait for some voice active audio to arrive
                    continue

                input_sample = input_bytes.copy()

                if duration > 3:
                    result, info = self.transcriber.transcribe(
                        input_sample, 
                        initial_prompt=None,
                        language='en',
                        # task=self.task,
                        vad_filter=True,
                        vad_parameters={"threshold": 0.5}
                    )
                    for segment in result:
                        print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))

                # if (duration > 10 and duration < 10.1) or (duration > 24.9 and duration < 25):
                #     print('save to file')

                #     t = threading.Thread(
                #         target=self.write_audio_frames_to_file,
                #         args=(
                #             input_sample.tobytes(),
                #             f"{self.n_audio_file}.wav",
                #         ),
                #     )
                #     t.start()
                #     self.n_audio_file += 1
                #     self.frames = b""

        finally:
            # stream.stop_stream()
            # stream.close()
            # audio.terminate()
            print('ended')

    async def start_websocket_server(self):
        async with websockets.serve(self.websocket_handler, '', 6006):
            await asyncio.Future()  # Keep the server running


if __name__ == '__main__':
    server = TranscriptionServer()
    asyncio.run(server.start_websocket_server())
