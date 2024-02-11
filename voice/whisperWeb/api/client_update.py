import os
import wave

import numpy as np
# import scipy
# import ffmpeg
import pyaudio
# import threading
# import textwrap
# import json
# import websocket
import uuid
# import time
from faster_whisper import WhisperModel

model_size = "base.en"
os.environ['KMP_DUPLICATE_LIB_OK']='True'

class UpdatedClient:
    INSTANCES = {}

    def __init__(
        self, host=None, port=None, is_multilingual=False, lang=None, translate=False, model_size="base.en"
    ):

        self.chunk = 1024 * 3
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.record_seconds = 60000
        self.recording = False
        self.multilingual = False
        self.language = None
        self.task = "transcribe"
        self.uid = str(uuid.uuid4())
        self.waiting = False
        self.last_response_recieved = None
        self.disconnect_if_no_response_for = 15
        self.multilingual = is_multilingual
        self.language = lang
        self.model_size = model_size
        self.server_error = False
        if translate:
            self.task = "translate"

        self.timestamp_offset = 0.0
        self.audio_bytes = None
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk,
        )
        # self.stream = self.p.open(format=FORMAT,
        #         channels=CHANNELS,
        #         rate=self.RATE,
        #         input=True,
        #         frames_per_buffer=CHUNK)

        self.model = WhisperModel(model_size, device="cpu", compute_type="int8")


        UpdatedClient.INSTANCES[self.uid] = self

        self.frames = b""
        print("[INFO]: * recording")


    
    @staticmethod
    def bytes_to_float_array(audio_bytes):
        """
        Convert audio data from bytes to a NumPy float array.
        
        It assumes that the audio data is in 16-bit PCM format. The audio data is normalized to 
        have values between -1 and 1.

        Args:
            audio_bytes (bytes): Audio data in bytes.

        Returns:
            np.ndarray: A NumPy array containing the audio data as float values normalized between -1 and 1.
        """
        raw_data = np.frombuffer(buffer=audio_bytes, dtype=np.int16)

        return raw_data.astype(np.float32) / 32768.0

    def send_packet_to_server(self, n_audio_file, message):
        """
        Send an audio packet to the server using WebSocket.

        Args:
            message (bytes): The audio data packet in bytes to be sent to the server.

        """
        try:
            print('send message to transcribe')
            # print(message)
            # frame_np = np.frombuffer(message, dtype=np.float32)
            self.write_output_recording(n_audio_file, message)

            # segments_all, info = self.model.transcribe(message, beam_size=5)
            # for segment in segments_all:
            #     print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))


            # self.client_socket.send(message, websocket.ABNF.OPCODE_BINARY)
        except Exception as e:
            print(e)


    def record(self):

        print('recording')
        n_audio_file = 0
        # if not os.path.exists("chunks"):
        #     os.makedirs("chunks", exist_ok=True)
        try:
            while True:
                # if not self.recording:
                #     break
                data = self.stream.read(self.chunk)
                self.frames += data

                audio_array = UpdatedClient.bytes_to_float_array(self.frames)
                # raw_data = np.frombuffer(buffer=self.frames, dtype=np.int16)
                duration = audio_array.shape[0] / self.rate
                print(duration)
                if duration < 1:
                    continue
                self.send_packet_to_server(n_audio_file, audio_array)

                # save frames if more than a minute
                if len(self.frames) > 60 * self.rate:
                    n_audio_file += 1
                    self.frames = b""
                    print('reset frame')

        except KeyboardInterrupt:
            if len(self.frames):
                n_audio_file += 1
            self.stream.stop_stream()
            self.stream.close()
            self.p.terminate()

    def write_output_recording(self, n_audio_file, frames, out_file="output_recording.wav"):
            """
            Combine and save recorded audio chunks into a single WAV file.
            
            The individual audio chunk files are expected to be located in the "chunks" directory. Reads each chunk 
            file, appends its audio data to the final recording, and then deletes the chunk file. After combining
            and saving, the final recording is stored in the specified `out_file`.


            Args:
                n_audio_file (int): The number of audio chunk files to combine.
                out_file (str): The name of the output WAV file to save the final recording.

            """
            input_files = [
                f"chunks/{i}.wav"
                for i in range(n_audio_file)
                if os.path.exists(f"chunks/{i}.wav")
            ]
            with wave.open(out_file, "wb") as wavfile:
                wavfile: wave.Wave_write
                wavfile.setnchannels(self.channels)
                wavfile.setsampwidth(2)
                wavfile.setframerate(self.rate)
                for in_file in input_files:
                    with wave.open(in_file, "rb") as wav_in:
                        while True:
                            data = wav_in.readframes(frames)
                            if data == b"":
                                break
                            wavfile.writeframes(data)
                    # remove this file
                    # os.remove(in_file)
            wavfile.close()

            
class TranscriptionClient:

    def __init__(self, host, port, is_multilingual=False, lang=None, translate=False, model_size="small"):
        self.client = UpdatedClient(host, port, is_multilingual, lang, translate, model_size)

    def __call__(self, audio=None, hls_url=None):

        print("[INFO]: Waiting for server ready ...")

        self.recording = True
        self.client.record()


if __name__ == "__main__":
    client = TranscriptionClient(
    "localhost",
    6006,
    is_multilingual=False,
    lang="en",
    translate=False,
    )
    client()



# File object has no read() method, or readable() returned False.