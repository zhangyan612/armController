import multiprocessing
from multiprocessing import Manager, Queue

# from transcription_server import TranscriptionServer
# from trt_server import TranscriptionServer
from transcription_server_update import TranscriptionServer
from llm_api_empty import EmptyLLM
from tts_service import WhisperSpeechTTS

# import logging
# logging.basicConfig(level=logging.DEBUG)


if __name__ == "__main__":
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
            transcription_queue,
            llm_queue,
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
