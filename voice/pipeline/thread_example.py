import threading
import time
from threading import Event
from multiprocessing import Queue

class VoiceToText:
    def run(self, llm_queue, tts_playing_event):
        for i in range(100):
            while tts_playing_event.is_set():
                time.sleep(0.1)
            print(f"send text data to llm: {i}")
            llm_queue.put(f"data {i}")
            time.sleep(5)

class LLMService:
    def run(self, llm_queue, audio_queue):
        while True:
            if not llm_queue.empty():
                data = llm_queue.get()
                print(f"LLM service received: {data}")
                # request to llm 
                time.sleep(1)
                audio_queue.put(data + " processed by OpenAILLM")

class EdgeTTS:
    def run(self, audio_queue, tts_playing_event):
        while True:
            if not audio_queue.empty():
                data = audio_queue.get()
                tts_playing_event.set()
                print(f"EdgeTTS received: {data}")
                time.sleep(2)
                tts_playing_event.clear()

if __name__ == "__main__":
    llm_queue = Queue()
    audio_queue = Queue()

    tts_playing_event = Event()

    recorder_server = VoiceToText()
    recorder_thread = threading.Thread(
        target=recorder_server.run,
        args=(
            llm_queue,
            tts_playing_event
        )
    )
    recorder_thread.start()

    llm_provider = LLMService()
    llm_thread = threading.Thread(
        target=llm_provider.run,
        args=(
            llm_queue,
            audio_queue,
        )
    )
    llm_thread.start()

    tts_runner = EdgeTTS()
    tts_thread = threading.Thread(target=tts_runner.run, args=(audio_queue,tts_playing_event))
    tts_thread.start()

    recorder_thread.join()
    llm_thread.join()
    tts_thread.join()
