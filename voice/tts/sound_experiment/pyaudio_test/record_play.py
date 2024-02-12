import pyaudio
import numpy as np

# Set up constants
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 1024
RECORD_SECONDS = 5

# Create a PyAudio instance
audio = pyaudio.PyAudio()


# import pyaudio
# pa = pyaudio.PyAudio()
# print(pa.get_default_output_device_info())


# Start recording
stream = audio.open(format=FORMAT, channels=CHANNELS,
                    rate=RATE, input=True,
                    frames_per_buffer=CHUNK)
print("Recording...")
frames = []

for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)
print("Finished recording")

# Stop recording
stream.stop_stream()
stream.close()
audio.terminate()

# # Start playback
# print("Playing back...")
# stream = audio.open(format=FORMAT, channels=CHANNELS,
#                     rate=RATE, output=True, output_device_index=3)
# for frame in frames:
#     stream.write(frame)
# stream.stop_stream()
# stream.close()
# audio.terminate()
# print("Playback finished")
