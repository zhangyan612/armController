import pyaudio
import os
from faster_whisper import WhisperModel
import numpy as np

FORMAT = pyaudio.paInt16
CHUNK = 4096
# Choose your channels
CHANNELS = 1
# RATE = 44100

model_size = "base.en"
os.environ['KMP_DUPLICATE_LIB_OK']='True'


class TranscriptionMic:
    def __init__(self):
        self.frames_np = None
        self.frames_offset = 0.0
        self.timestamp_offset = 0.0
        self.RATE = 44100

        self.p = pyaudio.PyAudio()

        # or run on CPU with INT8 - works on cpu - 1 min
        self.model = WhisperModel(model_size, device="cpu", compute_type="int8")

        # Open a streaming session
        self.stream = self.p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=self.RATE,
                        input=True,
                        frames_per_buffer=CHUNK)


    def add_frames(self, frame_np):
        """
        Add audio frames to the ongoing audio stream buffer.

        This method is responsible for maintaining the audio stream buffer, allowing the continuous addition
        of audio frames as they are received. It also ensures that the buffer does not exceed a specified size
        to prevent excessive memory usage.

        If the buffer size exceeds a threshold (45 seconds of audio data), it discards the oldest 30 seconds
        of audio data to maintain a reasonable buffer size. If the buffer is empty, it initializes it with the provided
        audio frame. The audio stream buffer is used for real-time processing of audio data for transcription.

        Args:
            frame_np (numpy.ndarray): The audio frame data as a NumPy array.

        """
        if self.frames_np is not None and self.frames_np.shape[0] > 45*self.RATE:
            self.frames_offset += 30.0
            self.frames_np = self.frames_np[int(30*self.RATE):]
        if self.frames_np is None:
            self.frames_np = frame_np.copy()
        else:
            self.frames_np = np.concatenate((self.frames_np, frame_np), axis=0)

    def transcribe(self, audioData):
        segments, info = self.model.transcribe(audioData, beam_size=5)
        print("Detected language '%s' with probability %f" % (info.language, info.language_probability))
        for segment in segments:
            print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))


    def record(self):
        try:
            while True:
                # Read a chunk of audio data
                data = self.stream.read(CHUNK)
                
                # frame_np = np.frombuffer(data, dtype=np.float32)
                raw_data = np.frombuffer(buffer=data, dtype=np.int16)
                raw_data = raw_data.astype(np.float32) / 32768.0
                self.add_frames(raw_data)

                if self.frames_np is None:
                    continue

                # clip audio if the current chunk exceeds 30 seconds, this basically implies that
                # no valid segment for the last 30 seconds from whisper
                if self.frames_np[int((self.timestamp_offset - self.frames_offset)*self.RATE):].shape[0] > 25 * self.RATE:
                    duration = self.frames_np.shape[0] / self.RATE
                    self.timestamp_offset = self.frames_offset + duration - 5
        
                samples_take = max(0, (self.timestamp_offset - self.frames_offset)*self.RATE)
                input_bytes = self.frames_np[int(samples_take):].copy()
                # print(int(samples_take))
                duration = input_bytes.shape[0] / self.RATE

                if duration<0.4:
                    continue

                input_sample = input_bytes.copy()
                # self.transcribe(self.frames_np)

                print(duration)
                # save this input sample to file
                # t = threading.Thread(
                #     target=self.write_audio_frames_to_file,
                #     args=(
                #         self.frames,
                #         f"chunks/{n_audio_file}.wav",
                #     ),
                # )
                # t.start()

                segments_all, info = self.model.transcribe(input_sample, beam_size=5)
                for segment in segments_all:
                    print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))


        except KeyboardInterrupt:
            self.stream.stop_stream()
            self.stream.close()
            self.p.terminate()

TranscriptionMic().record()