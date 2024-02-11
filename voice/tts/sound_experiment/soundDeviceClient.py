# import pyaudio
# import websocket
# import pickle

# # Constants for audio streaming
# CHUNK = 1024
# FORMAT = pyaudio.paInt16
# CHANNELS = 1
# RATE = 44100
# RECORD_SECONDS = 40

# # WebSocket server address
# HOST = 'ws://127.0.0.1:8080'

# import sounddevice as sd
# import numpy as np
# import websocket
# import threading
# import queue

# # Parameters
# device = 0  # id of the audio device by default
# window = 1000  # window for the data
# downsample = 1  # how much samples to drop
# channels = [1]  # a list of audio channels
# chunkSize = 4096  # size of audio chunks to send
# server_url = "ws://localhost:6006"  # replace with your server URL

# Update imports with:
import websocket as ws
import sounddevice as sd
# from uhlive.stream.recognition import Recognizer, Opened

# Append the following to the code above:
def stream_mic(socket):
    def callback(indata, frame_count, time_info, status):
        print('send data')
        socket.send_binary(bytes(indata))

    stream = sd.RawInputStream(
        callback=callback, channels=1, samplerate=8000, dtype="int16", blocksize=960
    )
    stream.start()
    return stream

server_url = "ws://localhost:6006"  # replace with your server URL

socket = ws.create_connection(server_url)
# Open a session
# Commands are sent as text frames
print("Streaming mic")

stream = stream_mic(socket)

