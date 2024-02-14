import pyaudio
import webrtcvad
import numpy as np
import torch
# from vad import VoiceActivityDetection

class VadCheck:
    def __init__(self) -> None:
        # Set up WebRTC VAD
        self.webrtc_vad_model = webrtcvad.Vad()
        self.webrtc_vad_model.set_mode(1)  # set aggressiveness mode, in [0, 3]
        self.CHUNK = 1024
        self.RATE = 16000
        # vad_threshold = 0.8
        # vad_model = VoiceActivityDetection()

        # Set up PyAudio
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=16000,
                        input=True,
                        frames_per_buffer=1024)

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

    def recording(self):

        while True:
            # Read chunk of audio data
            data = self.stream.read(self.CHUNK)
            voice_detected = self._is_webrtc_speech(data)
            if voice_detected:
                print("Voice detected!")
            else:
                print("No voice detected.")

            # frame_np = np.frombuffer(data, dtype=np.float32)
            # frame_ts = torch.from_numpy(frame_np.copy())
            # speech_prob = vad_model(frame_ts, RATE).item()
            # print(speech_prob)
            # if speech_prob < vad_threshold:
            #     print('no voice')
            # else:
            #     print("Voice detected!")


if __name__ == "__main__":
    vad = VadCheck()
    vad.recording()