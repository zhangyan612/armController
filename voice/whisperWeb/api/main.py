import multiprocessing
# import argparse
# import threading
# import ssl
# import time
# import sys
# import functools

from multiprocessing import Manager, Queue


from transcription_server import TranscriptionServer

from llm_api_empty import EmptyLLM
from tts_service import WhisperSpeechTTS


if __name__ == "__main__":
    # In general works 
    multiprocessing.set_start_method('spawn')
    
    lock = multiprocessing.Lock()
    
    manager = Manager()
    shared_output = manager.list()

    transcription_queue = Queue()
    llm_queue = Queue()
    audio_queue = Queue()


    whisper_server = TranscriptionServer()
    whisper_process = multiprocessing.Process(
        target=whisper_server.run,
        args=(
            "0.0.0.0",
            6006,
            # transcription_queue,
            # llm_queue,
            # args.whisper_tensorrt_path
        )
    )
    whisper_process.start()

    llm_provider = EmptyLLM()

    llm_process = multiprocessing.Process(
        target=llm_provider.run,
        args=(
            transcription_queue,
            llm_queue,
            audio_queue,
        )
    )

    llm_process.start()

    # audio process
    tts_runner = WhisperSpeechTTS()
    tts_process = multiprocessing.Process(target=tts_runner.run, args=("0.0.0.0", 8888, audio_queue))
    tts_process.start()

    llm_process.join()
    whisper_process.join()
    tts_process.join()
