from faster_whisper import WhisperModel
# import datetime
import time
import os

model_size = "base.en"

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
start = time.time()

# or run on CPU with INT8 - works on cpu - 1 min
model = WhisperModel(model_size, device="cpu", compute_type="int8")
# wav_file_path = "D:\Download\sample1.flac"
laptop_file_path = "D:\Robot/armController/20240204-234829.wav"
jetson_path = '/home/wheeltec/armController/20240219-192946.wav'

segments, info = model.transcribe(jetson_path, beam_size=5)

print("Detected language '%s' with probability %f" % (info.language, info.language_probability))

for segment in segments:
    print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))

infer_time = time.time() - start

print(f"Whisper inference time {infer_time}\n")

# laptop
# Whisper inference time 4.273757696151733