import asyncio
import time
import os
import vlc
import edge_tts
import json
import persistqueue
import time
import threading
import queue

q = persistqueue.SQLiteQueue('audio', auto_commit=True)


# def subscribe_to_change(key):
#     filename = 'state.json'
#     last_modified = os.path.getmtime(filename)
#     last_value = None
#     while True:
#         if os.path.getmtime(filename) != last_modified:
#             last_modified = os.path.getmtime(filename)
#             with open(filename, 'r') as f:
#                 new_contents = json.load(f)
#             new_value = new_contents.get(key, None)
#             if new_value is not None and new_value != last_value:
#                 print(f"{key}: {new_value}")

#                 loop = asyncio.get_event_loop_policy().get_event_loop()
#                 loop.run_until_complete(generate_voice(new_value))

#                 last_value = new_value
#         time.sleep(1)

# Create a queue for audio files
audio_queue = queue.Queue()

def playSound():
    while True:
        file = audio_queue.get()
        p = vlc.MediaPlayer(file)
        p.play()
        time.sleep(1)
        while p.is_playing():
            time.sleep(1)
        os.remove(file)
        audio_queue.task_done()

async def generate_voice(text="Hello this is a test run", voice="en-US-SteffanNeural"):
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    cwd = os.getcwd()
    output_file = os.path.join(cwd, f"{timestamp}.wav")
    print(output_file)
    communicate = edge_tts.Communicate(text, voice)
    with open(output_file, "wb") as file:
        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                file.write(chunk["data"])
            elif chunk["type"] == "WordBoundary":
                print(f"WordBoundary: {chunk}")

    # Add the generated audio file to the queue
    audio_queue.put(output_file)

def runQueue():
    # Start the audio player thread
    threading.Thread(target=playSound, daemon=True).start()

    while True:
        msg = q.get()
        loop = asyncio.get_event_loop_policy().get_event_loop()
        loop.run_until_complete(generate_voice(msg))
        time.sleep(0.1)

if __name__ == "__main__":
    runQueue()

    # subscribe_to_change('audio')
    # loop = asyncio.get_event_loop_policy().get_event_loop()
    # try:
    #     loop.run_until_complete(generate_voice('This is a test.'))
    # finally:
    #     loop.close()


