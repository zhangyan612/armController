from faster_whisper import WhisperModel
# import datetime

model_size = "base.en"

import os

os.environ['KMP_DUPLICATE_LIB_OK']='True'

# works on cpu
# for gpu
# Purfview's whisper-standalone-win provides the required NVIDIA libraries for Windows & Linux in a single archive. Decompress the archive and place the libraries in a directory included in the PATH.


# print(datetime.datetime.now())

# Run on GPU with FP16
# model = WhisperModel(model_size, device="cuda", compute_type="float16")

# or run on GPU with float32- works but much slower
# model = WhisperModel(model_size, device="cuda", compute_type="float32")

# run on GPU with auto - 5 mins
# model = WhisperModel(model_size, device="cuda", compute_type="auto")

# or run on GPU with INT8 - not working
# model = WhisperModel(model_size, device="cuda", compute_type="int8_float16")

# or run on CPU with INT8 - works on cpu - 1 min
model = WhisperModel(model_size, device="cpu", compute_type="int8")
wav_file_path = "D:\Download\sample1.flac"

segments, info = model.transcribe(wav_file_path, beam_size=5)

print("Detected language '%s' with probability %f" % (info.language, info.language_probability))

for segment in segments:
    print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))
    
# print(datetime.datetime.now())
