import time
import logging
import asyncio
import os
import edge_tts
import vlc 
import json

class EdgeTTS:
    def __init__(self):
        self.last_llm_response = None

    def playSound(self, file):
        p = vlc.MediaPlayer(file)
        p.play()
        time.sleep(1)
        while p.is_playing():
            time.sleep(1)

    async def generate_voice(self, text="Hello this is a test run", voice="en-US-SteffanNeural"):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        cwd = os.getcwd()
        output_file = os.path.join(cwd, f"{timestamp}.wav")
        # print(output_file)
        communicate = edge_tts.Communicate(text, voice)
        with open(output_file, "wb") as file:
            async for chunk in communicate.stream():
                if chunk["type"] == "audio":
                    file.write(chunk["data"])
                elif chunk["type"] == "WordBoundary":
                    print(f"WordBoundary: {chunk}")
        return output_file

    def run(self, audio_queue=None, tts_playing_event=None):
        self.eos = False
        self.output_audio = None

        while True:
            llm_output = audio_queue.get()
            if audio_queue.qsize() != 0:
                continue
                        
            # only process if the output updated
            if self.last_llm_response != llm_output.strip():
                try:
                    start = time.time()
                    new_loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(new_loop)
                    audio_file_path = new_loop.run_until_complete(self.generate_voice(llm_output.strip()))
                    inference_time = time.time() - start
                    logging.info(f"[TTS Service]: TTS inference done in {inference_time} ms.")
                    self.last_llm_response = llm_output.strip()
                    self.output_audio = audio_file_path
                except TimeoutError:
                    pass
                
            if self.output_audio is not None:
                try:
                    tts_playing_event.set()

                    self.playSound(self.output_audio)
                    logging.info(f"[TTS Service]: playing audio file")
                    if os.path.isfile(self.output_audio):
                        os.remove(self.output_audio)
                        self.output_audio = None

                    tts_playing_event.clear()

                except Exception as e:
                    logging.error(f"[TTS ERROR]: Audio error: {e}")

def subscribe_to_change(key, audio_queue):
    filename = 'state.json'
    last_modified = os.path.getmtime(filename)
    last_value = None
    while True:
        if os.path.getmtime(filename) != last_modified:
            last_modified = os.path.getmtime(filename)
            with open(filename, 'r') as f:
                new_contents = json.load(f)
            new_value = new_contents.get(key, None)
            if new_value is not None and new_value != last_value:
                print(f"{key}: {new_value}")
                audio_queue.put(new_value)
                last_value = new_value
        time.sleep(1)

if __name__ == "__main__":
    import multiprocessing
    from multiprocessing import Queue, Event

    audio_queue = Queue()
    tts_playing_event = Event()

    tts_runner = EdgeTTS()
    tts_thread = multiprocessing.Process(target=tts_runner.run, args=(audio_queue,tts_playing_event))
    tts_thread.start()

    # listen to file change and add to queue
    subscribe_to_change('audio', audio_queue)

    # listen to file change and add to queue
    # audio_queue.put('test test')


