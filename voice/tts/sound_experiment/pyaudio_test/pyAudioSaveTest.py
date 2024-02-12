import pyaudio
import wave
import time

CHUNK = 1024 * 3
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "output"

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

print("* recording")

frames = []

try:
    file_number = 0
    while True:
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            print(i)
            data = stream.read(CHUNK)
            frames.append(data)
        
        # Save the last 5 seconds of audio data to a .wav file
        wf = wave.open(WAVE_OUTPUT_FILENAME + str(file_number) + ".wav", 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames[-int(RATE / CHUNK * RECORD_SECONDS):]))
        wf.close()
        
        print("* saved to " + WAVE_OUTPUT_FILENAME + str(file_number) + ".wav")
        
        # Increment the file number
        file_number += 1
        
        # Wait for 5 seconds
        time.sleep(5)
except KeyboardInterrupt:
    pass

print("* done recording")

stream.stop_stream()
stream.close()
p.terminate()
