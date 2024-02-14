import pyaudio
import os
from faster_whisper import WhisperModel
import numpy as np
import webrtcvad

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
        self.webrtc_vad_model = webrtcvad.Vad()
        self.webrtc_vad_model.set_mode(1)  # set aggressiveness mode, in [0, 3]

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
        if self.frames_np is not None and self.frames_np.shape[0] > 45*self.RATE:
            self.frames_offset += 30.0
            self.frames_np = self.frames_np[int(30*self.RATE):]
        if self.frames_np is None:
            self.frames_np = frame_np.copy()
        else:
            self.frames_np = np.concatenate((self.frames_np, frame_np), axis=0)

    def _is_webrtc_speech(self, data, all_frames_must_be_true=False):
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

    def transcribe(self, input_sample):
        segments, info = self.model.transcribe(input_sample, beam_size=5)
        print("Detected language '%s' with probability %f" % (info.language, info.language_probability))
        for segment in segments:
            print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))

    def record(self):
        try:
            while True:
                # Read a chunk of audio data
                data = self.stream.read(CHUNK)

                voice_detected = self._is_webrtc_speech(data)
                if voice_detected:
                    print("Voice detected!")
                    # frame_np = np.frombuffer(data, dtype=np.float32)
                    raw_data = np.frombuffer(buffer=data, dtype=np.int16)
                    raw_data = raw_data.astype(np.float32) / 32768.0
                    self.add_frames(raw_data)

                    if self.frames_np is None:
                        continue

                else:
                    if self.on_recording:
                        print("Voice ended, transcribing...")
                        # concatenate frames
                        
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
                        print('audio duration: ', duration)
                        # transcribe audio data
                        self.transcribe(input_sample)
                        # save this input sample to file
                        # t = threading.Thread(
                        #     target=self.write_audio_frames_to_file,
                        #     args=(
                        #         self.frames,
                        #         f"chunks/{n_audio_file}.wav",
                        #     ),
                        # )
                        # t.start()
                        # clear frames
                        self.frames_np = []
                        self.on_recording = False
                    else:
                        print("No voice detected.")
                

        except KeyboardInterrupt:
            self.stream.stop_stream()
            self.stream.close()
            self.p.terminate()

TranscriptionMic().record()