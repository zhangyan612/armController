import sounddevice as sd
import numpy as np

# Buffer size
chunk_size = 4096

# Buffer
buffer = np.zeros(chunk_size, dtype=np.float32)

# Buffer pointer
buffer_pointer = 0

def audio_callback(indata, frames, time, status):
    global buffer, buffer_pointer

    # Process audio input
    for sample in indata:
        buffer[buffer_pointer] = sample[0]
        buffer_pointer += 1

        # If buffer is full
        if buffer_pointer >= chunk_size:
            # Reset buffer pointer
            buffer_pointer = 0
            print('buffer full')

# Create stream
stream = sd.InputStream(callback=audio_callback)
with stream:
    while True:
        pass
