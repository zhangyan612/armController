import threading
import time
from threading import Event
from multiprocessing import Queue
from voice_text_service import VoiceToText
# from lepton_LLM import leptonLLM
from fake_LLM import leptonLLM
from llm_memory import LLMMemory
# from edge_TTS_MuitiProcess import EdgeTTS
# import json 
import nltk
import persistqueue

# class VoiceToText:
#     def run(self, llm_queue, tts_playing_event):
#         for i in range(100):
#             while tts_playing_event.is_set():
#                 time.sleep(0.1)
#             print(f"send text data to llm: {i}")
#             llm_queue.put(f"data {i}")
#             time.sleep(5)

# file based state mgt
# def updateState(key, value):
#     with open('state.json', 'w') as f:
#         json.dump({'time': time.ctime(), key: value}, f)


def split_paragraph(paragraph):
    # Ensure NLTK sentence tokenizer is downloaded
    nltk.download('punkt', quiet=True)
    
    # Use NLTK's sent_tokenize function to split the paragraph
    sentences = nltk.sent_tokenize(paragraph)
    
    # Split sentences further if they exceed 100 characters
    final_sentences = []
    for sentence in sentences:
        if len(sentence) > 100:
            # Split by comma
            comma_splits = sentence.split(',')
            temp_sentence = ''
            for split in comma_splits:
                if len(temp_sentence) + len(split) < 100:
                    temp_sentence += split + ','
                else:
                    final_sentences.append(temp_sentence.strip(','))
                    temp_sentence = split + ','
            final_sentences.append(temp_sentence.strip(','))
        else:
            final_sentences.append(sentence)
    
    return final_sentences

class LLMService:
    # def __init__(self) -> None:
    #     import persistqueue
    #     self.queue = persistqueue.SQLiteQueue('audio', auto_commit=True)

    def run(self, llm_queue, audio_queue=None):
        memory = LLMMemory('You are a robot')
        llm = leptonLLM()
        queue = persistqueue.SQLiteQueue('audio', auto_commit=True)

        while True:
            if not llm_queue.empty():
                data = llm_queue.get()
                print(f"LLM service received: {data}")

                prompt = memory.add_message_history('user', data)
                response = llm.llm_request(prompt)
                memory.add_message_history('assistant', response)
                sentences = split_paragraph(response)
                # audio_queue.put(response)
                # write to a file instead of queue
                # updateState('audio', response)
                # split setense to shorter if too long 
                for s in sentences:
                    queue.put(s)


# class EdgeTTS:
#     def run(self, audio_queue, tts_playing_event):
#         while True:
#             if not audio_queue.empty():
#                 data = audio_queue.get()
#                 tts_playing_event.set()
#                 print(f"EdgeTTS received: {data}")
#                 time.sleep(2)
#                 tts_playing_event.clear()

if __name__ == "__main__":
    llm_queue = Queue()
    # audio_queue = Queue()

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
            # audio_queue,
        )
    )
    llm_thread.start()

    # tts_runner = EdgeTTS()
    # tts_thread = threading.Thread(target=tts_runner.run, args=(audio_queue,tts_playing_event))
    # tts_thread.start()

    recorder_thread.join()
    llm_thread.join()
    # tts_thread.join()
