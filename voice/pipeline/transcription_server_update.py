import time
import json

import torch
import numpy as np
import time
from transcriber import WhisperModel
import os
import platform
import time
import threading
import json
import textwrap
# import pyaudio
import logging
logging.basicConfig(level = logging.INFO)

import numpy as np
import time
import queue
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

class TranscriptionServer:

    def __init__(self):
        # voice activity detection model
        self.RATE = 16000
        self.CHUNK = 1024 * 3
        self.clients = {}
        self.websockets = {}
        self.clients_start_time = {}
        self.max_clients = 4
        self.max_connection_time = 600
        self.transcriber = None
        # self.p = pyaudio.PyAudio()
        # self.stream = self.p.open(format=pyaudio.paInt16,
        #                 channels=1,
        #                 rate=self.RATE,
        #                 input=True,
        #                 frames_per_buffer=self.CHUNK)

    def assign_client(self, options, transcription_queue=None, llm_queue=None):
        client = ServeClient(
            multilingual=options["multilingual"],
            language=options["language"],
            task=options["task"],
            client_uid=options["uid"],
            transcription_queue=transcription_queue,
            llm_queue=llm_queue,
            transcriber=self.transcriber
        )

        self.clients = client
        return client

    def handle_transcriber(self):
        device = "cpu"
        compute = "int8"
        if self.transcriber is None:
            if torch.cuda.is_available() and 'ubuntu' in platform.platform().lower():
                device = "cuda"
                compute = "float16"

            self.transcriber = WhisperModel(
                model_size_or_path="base.en",
                device=device,
                compute_type=compute,
                local_files_only=False,
            )
        return device, compute


    def run(self, transcription_queue=None, llm_queue=None, tts_playing_event=None):

        device, compute = self.handle_transcriber()

        no_voice_activity_chunks = 0

        while True:
            if not tts_playing_event.is_set():
                time.sleep(4)
                print('test')
                # data = self.stream.read(self.CHUNK)
                # frames = np.frombuffer(buffer=data, dtype=np.int16)
                # # Convert s16 back to f32.
                # frame_np = frames.astype(np.float32) / 32768.0

                # self.clients.add_frames(frame_np)



class ServeClient:
    RATE = 16000
    SERVER_READY = "SERVER_READY"
    DISCONNECT = "DISCONNECT"

    def __init__(
        self,
        task="transcribe",
        device=None,
        multilingual=False,
        language=None, 
        client_uid=None,
        transcription_queue=None,
        llm_queue=None,
        transcriber=None
        ):
        if transcriber is None:
            raise ValueError("Transcriber is None.")
        self.transcriber = transcriber
        self.client_uid = client_uid
        self.transcription_queue = transcription_queue
        self.llm_queue = llm_queue
        self.data = b""
        self.frames = b""
        self.language = language if multilingual else "en"
        self.task = task
        self.last_prompt = None
        
        
        self.timestamp_offset = 0.0
        self.frames_np = None
        self.frames_offset = 0.0

        self.exit = False
        self.transcript = []
        self.prompt = None
        self.segment_inference_time = []

        self.text = []
        self.current_out = ''
        self.prev_out = ''
        self.t_start=None

        self.same_output_threshold = 0
        self.show_prev_out_thresh = 5   # if pause(no output from whisper) show previous output for 5 seconds
        self.add_pause_thresh = 4       # add a blank to segment list as a pause(no speech) for 3 seconds
        self.sleep_duration = 0.4
        self.send_last_n_segments = 10

        # text formatting
        self.wrapper = textwrap.TextWrapper(width=50)
        self.pick_previous_segments = 2

        # threading
        self.lock = threading.Lock()
        self.eos = False
        self.trans_thread = threading.Thread(target=self.speech_to_text)
        self.trans_thread.start()

    
    def add_frames(self, frame_np):
        self.lock.acquire()
        if self.frames_np is not None and self.frames_np.shape[0] > 45*self.RATE:
            self.frames_offset += 30.0
            self.frames_np = self.frames_np[int(30*self.RATE):]
        if self.frames_np is None:
            self.frames_np = frame_np.copy()
        else:
            self.frames_np = np.concatenate((self.frames_np, frame_np), axis=0)
        self.lock.release()


    def speech_to_text(self):
        while True:
            try:
                input_sample = self.frame_np.copy()
                start = time.time()
                logging.info('run transcription')
                # whisper transcribe with prompt
                result, info = self.transcriber.transcribe(
                    input_sample, 
                    initial_prompt=None,
                    language=self.language,
                    task=self.task,
                    vad_filter=True,
                    vad_parameters={"threshold": 0.5}
                )

                infer_time = time.time() - start
                self.segment_inference_time.append(infer_time)
                print(result)
                
            except Exception as e:
                logging.error(f"[Transcription ERROR]: {e}")
                time.sleep(0.01)
    

if __name__ == "__main__":
    import multiprocessing
    from multiprocessing import Event, Queue
    tts_playing_event = Event()
    transcription_queue = Queue()
    llm_queue = Queue()

    # server = TranscriptionServer()
    # # 'NoneType' object has no attribute 'put'  -- need transcription queue
    # server.run("0.0.0.0", 6006, None, None, tts_playing_event)

    whisper_server = TranscriptionServer()
    whisper_process = multiprocessing.Process(  
        target=whisper_server.run,
        args=(
            transcription_queue,
            llm_queue,
            tts_playing_event
        )
    )
    whisper_process.start()
