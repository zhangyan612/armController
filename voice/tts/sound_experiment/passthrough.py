import sounddevice as sd
import numpy  # Make sure NumPy is loaded before it is used in the callback
assert numpy  # avoid "imported but unused" message (W0611)
import torch
import time 

channels = 2

eos = False

# Buffer size
chunk_size = 4096

# Buffer
buffer = numpy.zeros(chunk_size, dtype=numpy.float32)

# Buffer pointer
buffer_pointer = 0
buffer_pointer = 0


# def voice_activity_detection(frame_np, no_voice_activity_chunks, eos):
#     try:
#         speech_prob = self.vad_model(torch.from_numpy(frame_np.copy()), self.RATE).item()
#         if speech_prob < self.vad_threshold:
#             no_voice_activity_chunks += 1
#             if no_voice_activity_chunks > 3:
#                 if eos:
#                     eos = True
#                 time.sleep(0.1)
#             return no_voice_activity_chunks, False
#         no_voice_activity_chunks = 0
#         eos = False
#         return no_voice_activity_chunks, True
#     except Exception as e:
#         print(e)
#         return no_voice_activity_chunks, False

def callback(indata, outdata, frames, time, status):
    global buffer_pointer

    if status:
        print(status)

    # outdata[:] = indata

    # # Process audio input
    # for sample in indata:
    #     buffer[buffer_pointer] = sample[0]
    #     buffer_pointer += 1
    #     print(buffer_pointer)
    #     print(chunk_size)

    #     # If buffer is full
    #     if buffer_pointer >= chunk_size:
    #         # Reset buffer pointer
    #         buffer_pointer = 0
    #         print('buffer full')
    #         outdata[:] = indata
    # time.sleep(5)
    outdata[:] = indata

    # no_voice_activity_chunks, continue_processing = voice_activity_detection(frame_np, no_voice_activity_chunks, eos)
    # if not continue_processing:
    #     continue


try:
    with sd.Stream(channels=channels, callback=callback):
        print('#' * 80)
        print('press Return to quit')
        print('#' * 80)
        input()
except KeyboardInterrupt:
    # parser.exit('')
    print('keyboard interrupt')
except Exception as e:
    # parser.exit(type(e).__name__ + ': ' + str(e))
    print('exception')

