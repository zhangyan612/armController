import sounddevice as sd
import numpy as np
import websocket
import threading
import queue

# Parameters
device = 0  # id of the audio device by default
# window = 1000  # window for the data
downsample = 1  # how much samples to drop
channels = [1]  # a list of audio channels
chunkSize = 4096  # size of audio chunks to send
server_url = "ws://localhost:6006"  # replace with your server URL

# Initialize queue
q = queue.Queue()

# Get device info and sample rate
device_info = sd.query_devices(device, 'input')
samplerate = device_info['default_samplerate']

# WebSocket client
class AudioWebSocketClient(websocket.WebSocketApp):
    def __init__(self, url, protocols=None, extensions=None, heartbeat_freq=None, sslopt=None,
                 headers=None, log_level=None):
        super().__init__(url, protocols, extensions, heartbeat_freq, sslopt, headers, log_level)
        self.buffer = np.zeros(chunkSize)
        self.bufferPointer = 0

    def on_open(self):
        print("WebSocket connection opened.")

    def send_audio_data(self, data):
        data = data.flatten()
        while len(data) > 0:
            space_left = chunkSize - self.bufferPointer
            if len(data) <= space_left:
                self.buffer[self.bufferPointer:self.bufferPointer + len(data)] = data
                self.bufferPointer += len(data)
                data = []
            else:
                self.buffer[self.bufferPointer:] = data[:space_left]
                self.send(self.buffer.tobytes())
                self.bufferPointer = 0
                data = data[space_left:]

    def on_close(self):
        print("WebSocket connection closed.")

# Initialize WebSocket client
ws_client = AudioWebSocketClient(server_url)

# Start WebSocket connection in a separate thread
ws_thread = threading.Thread(target=ws_client.run_forever)
ws_thread.start()

# Audio callback
def audio_callback(indata, frames, time, status):
    q.put(indata[::downsample, [0]])

# Start audio stream
stream = sd.InputStream(device=device, channels=max(channels), samplerate=samplerate, callback=audio_callback)
stream.start()

# Main loop
try:
    while True:
        data = q.get()
        ws_client.send_audio_data(data)
except KeyboardInterrupt:
    pass
finally:
    stream.stop()
    stream.close()
    ws_client.close()
