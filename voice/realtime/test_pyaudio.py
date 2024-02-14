import pyaudio
# import pvporcupine

input_device_index = 0


# porcupine = pvporcupine.create(
#     keywords=wake_words_list,
#     sensitivities=sensitivity_list
# )
# buffer_size = porcupine.frame_length
# sample_rate = porcupine.sample_rate


SAMPLE_RATE = 16000
BUFFER_SIZE = 512
CHUNK = 4096

sample_rate =SAMPLE_RATE
buffer_size = BUFFER_SIZE

audio_interface = pyaudio.PyAudio()
stream = audio_interface.open(rate=sample_rate,
                                format=pyaudio.paInt16,
                                channels=1,
                                input=True,
                                frames_per_buffer=buffer_size,
                                input_device_index=input_device_index,
                                )

file_number = 0
while True:
    for i in range(0, int(10)):
        data = stream.read(CHUNK)
        print(data)
