import multiprocessing
from multiprocessing import Manager, Queue, Event

# from transcription_server import TranscriptionServer
# from trt_server import TranscriptionServer
from transcription_server_update import TranscriptionServer
from llm_service import LLMService
# from tts_service import WhisperSpeechTTS
# from tts_service_file import WhisperSpeechTTS

from edge_tts_service import EdgeTTSService

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

    # Event to control transcription service listening
    tts_playing_event = Event()

    whisper_server = TranscriptionServer()
    whisper_process = multiprocessing.Process(  
        target=whisper_server.run,
        args=(
            "0.0.0.0",
            6006,
            transcription_queue,
            llm_queue,
            tts_playing_event
        )
    )
    whisper_process.start()

    llm_provider = LLMService()

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
    tts_runner = EdgeTTSService()
    tts_process = multiprocessing.Process(
        target=tts_runner.run, 
        args=(
            "0.0.0.0", 
            8888, 
            audio_queue,
            tts_playing_event
        )
    )
    tts_process.start()

    llm_process.join()
    whisper_process.join()
    tts_process.join()


# INFO:faster_whisper:Processing audio with duration 00:01.280
# INFO:faster_whisper:VAD filter removed 00:00.000 of audio
# DEBUG:faster_whisper:VAD filter kept the following audio segments: [00:00.000 -> 00:01.280]
# ERROR:root:[ERROR]: 'WhisperModel' object has no attribute 'model'