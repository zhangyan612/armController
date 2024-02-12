import pyaudio
import numpy as np
import asyncio
import websockets
import threading
import wave 

CHUNK_SIZE = 1024 * 3
frames = []
n_audio_file = 0

audio = pyaudio.PyAudio()

stream = audio.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=16000,
                    output=True,
                    frames_per_buffer=CHUNK_SIZE)



def write_audio_frames_to_file(frames, file_name, rate=None):
    """
    Write audio frames to a WAV file.

    The WAV file is created or overwritten with the specified name. The audio frames should be 
    in the correct format and match the specified channel, sample width, and sample rate.

    Args:
        frames (bytes): The audio frames to be written to the file.
        file_name (str): The name of the WAV file to which the frames will be written.

    """
    with wave.open(file_name, "wb") as wavfile:
        rate = 16000
        wavfile: wave.Wave_write
        wavfile.setnchannels(1)
        wavfile.setsampwidth(2)
        wavfile.setframerate(rate)
        wavfile.writeframes(frames)


async def websocket_handler(websocket, path):
    try:
        async for message in websocket:
            print(message)
            global frames, n_audio_file
            frames += message

            # audio_data = np.frombuffer(message, dtype=np.int16)
            # stream.write(audio_data.tobytes())
            #stream.write(message)
            if len(frames) > 30000: # * self.rate:
                print('save to file')

                t = threading.Thread(
                    target=write_audio_frames_to_file,
                    args=(
                        frames,
                        f"{n_audio_file}.wav",
                    ),
                )
                t.start()
                n_audio_file += 1
                frames = b""

    finally:
        stream.stop_stream()
        stream.close()
        audio.terminate()

async def start_websocket_server():
    async with websockets.serve(websocket_handler, '', 8888):
        await asyncio.Future()  # Keep the server running


if __name__ == '__main__':
    asyncio.run(start_websocket_server())
