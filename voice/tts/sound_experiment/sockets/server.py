import asyncio
import websockets
import numpy as np
from faster_whisper import WhisperModel
import os

os.environ['KMP_DUPLICATE_LIB_OK']='True'
model_size = "base.en"


async def server(websocket, path):
    audio_data = await websocket.recv()

    frame_np = np.frombuffer(audio_data, dtype=np.float32)

    print("Received audio data")
    # print(frame_np)
    model = WhisperModel(model_size, device="cpu", compute_type="int8")
    # segments, info = model.transcribe(wav_file_path, beam_size=5)
    segments, info = model.transcribe(
        frame_np, 
        initial_prompt=None,
        language='en',
        vad_filter=True,
        vad_parameters={"threshold": 0.5}
    )
    print(info)
    print(segments)

    while True:
        try:
            segment = next(segments)
            print(segment)
        except StopIteration:
            break

    for segment in segments:
        print(segment)
        print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))



start_server = websockets.serve(server, "localhost", 6006)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
