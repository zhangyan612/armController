import pyaudio
import webrtcvad
import numpy as np
import os
from faster_whisper import WhisperModel
import threading
import wave

os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

class VoiceRecorder:
    def __init__(self) -> None:
        # Set up WebRTC VAD
        self.webrtc_vad_model = webrtcvad.Vad()
        self.webrtc_vad_model.set_mode(1)  # set aggressiveness mode, in [0, 3]
        self.CHUNK = 1024 * 3
        self.RATE = 16000
        self.model_size = 'base.en'
        self.frames_np = None
        self.on_recording = False
        self.frames_offset = 0.0
        self.timestamp_offset = 0.0

        self.frames = []
        # Set up PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=self.RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)
        
        self.model = WhisperModel(self.model_size, device="cpu", compute_type="int8")

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

    def transcribe(self, input_audio):
        # this works 
        audio = np.frombuffer(buffer=input_audio, dtype=np.int16)
        # Convert s16 back to f32.
        audio = audio.astype(np.float32) / 32768.0

        segments, info = self.model.transcribe(audio, beam_size=5, vad_filter=True, vad_parameters={"threshold": 0.5})
        print("Detected language '%s' with probability %f" % (info.language, info.language_probability))
        for segment in segments:
            print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))

    def write_audio_frames_to_file(self, frames, file_name):
        with wave.open(file_name, "wb") as wavfile:
            wavfile: wave.Wave_write
            wavfile.setnchannels(1)
            wavfile.setsampwidth(2)
            wavfile.setframerate(self.RATE)
            wavfile.writeframes(frames)

    def transcribe_from_File(self, frames, file_name):
        self.write_audio_frames_to_file(frames, file_name)
        self.transcribe(file_name) # need to remove conversion

    def recording(self):
        n_audio_file = 0

        while True:
            # Read chunk of audio data
            data = self.stream.read(self.CHUNK)
            voice_detected = self._is_webrtc_speech(data)
            if voice_detected:
                print("Voice detected!")
                # add data to frames
                self.frames.append(data)

                # works too
                # raw_data = np.frombuffer(buffer=data, dtype=np.int16)
                # raw_data = raw_data.astype(np.float32) / 32768.0
                # self.add_frames(raw_data)
                self.on_recording = True
            else:
                if self.on_recording:
                    print("Voice ended, transcribing...")
                    # concatenate frames
                    audio_data = b''.join(self.frames)

                    t = threading.Thread(
                        target=self.transcribe,
                        args=(
                            audio_data,
                        ),
                    )
                    t.start()

                    # save to file
                    # t = threading.Thread(
                    #     target=self.transcribe_from_File,
                    #     args=(
                    #         audio_data,
                    #         f"chunks/{n_audio_file}.wav",
                    #     ),
                    # )
                    # t.start()
                    # n_audio_file += 1

                    # clear frames
                    self.frames = []
                    # self.frames_np = None
                    self.on_recording = False
                else:
                    print("No voice detected.")
                    # if no voice detected for 5 seconds, print whole setense


if __name__ == "__main__":
    vad = VoiceRecorder()
    vad.recording()