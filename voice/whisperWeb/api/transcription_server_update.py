import time
import json

import logging
logging.basicConfig(level = logging.INFO)

from websockets.sync.server import serve
from vad import VoiceActivityDetection

import torch
import numpy as np
import time
from transcriber import WhisperModel
import os
import functools
import platform
from transcription_server_client import ServeClient

os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

class TranscriptionServer:
    """
    Represents a transcription server that handles incoming audio from clients.

    Attributes:
        RATE (int): The audio sampling rate (constant) set to 16000.
        vad_model (torch.Module): The voice activity detection model.
        vad_threshold (float): The voice activity detection threshold.
        clients (dict): A dictionary to store connected clients.
        websockets (dict): A dictionary to store WebSocket connections.
        clients_start_time (dict): A dictionary to track client start times.
        max_clients (int): Maximum allowed connected clients.
        max_connection_time (int): Maximum allowed connection time in seconds.
    """

    RATE = 16000

    def __init__(self):
        # voice activity detection model

        self.clients = {}
        self.websockets = {}
        self.clients_start_time = {}
        self.max_clients = 4
        self.max_connection_time = 600
        self.transcriber = None

    def get_wait_time(self):
        """
        Calculate and return the estimated wait time for clients.

        Returns:
            float: The estimated wait time in minutes.
        """
        wait_time = None

        for k, v in self.clients_start_time.items():
            current_client_time_remaining = self.max_connection_time - (time.time() - v)

            if wait_time is None or current_client_time_remaining < wait_time:
                wait_time = current_client_time_remaining

        return wait_time / 60

    def options_processing(self, websocket):
        options = websocket.recv()
        options = json.loads(options)
        return options

    def handle_client_queue(self, websocket, options):
        if len(self.clients) >= self.max_clients:
            logging.warning("Client Queue Full. Asking client to wait ...")
            wait_time = self.get_wait_time()
            response = {
                "uid": options["uid"],
                "status": "WAIT",
                "message": wait_time,
            }
            websocket.send(json.dumps(response))
            websocket.close()
            del websocket
            return

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


    def assign_client(self, websocket, options, transcription_queue=None, llm_queue=None):
        client = ServeClient(
            websocket,
            multilingual=options["multilingual"],
            language=options["language"],
            task=options["task"],
            client_uid=options["uid"],
            transcription_queue=transcription_queue,
            llm_queue=llm_queue,
            transcriber=self.transcriber
        )

        self.clients[websocket] = client
        self.clients_start_time[websocket] = time.time()
        return client

    def frame_processing(self, websocket, client):
        try:
            frame_data = websocket.recv()
            frame_np = np.frombuffer(frame_data, dtype=np.float32)
            return frame_data, frame_np
        except Exception as e:
            logging.error(e)
            del websocket
            return


    def voice_activity_detection(self, frame_np, no_voice_activity_chunks, websocket):
        """Detects voice activity in an audio frame.
    
        Uses a pretrained Voice Activity Detection (VAD) model to determine if an audio 
        frame contains speech. Keeps track of consecutive silent frames and sets end-of-speech
        flag on client if threshold is exceeded. Returns number of silent frames and a 
        flag indicating if frame contains speech.
        
        Args:
            frame_np: Numpy array containing audio frame data.
            no_voice_activity_chunks: Tracker for number of consecutive silent frames.
            websocket: Websocket connection to client.
        
        Returns:
            no_voice_activity_chunks: Updated count of consecutive silent frames.
            continue_processing: Flag indicating if frame contains speech.
        """
        try:
            speech_prob = self.vad_model(torch.from_numpy(frame_np.copy()), self.RATE).item()
            if speech_prob < self.vad_threshold:
                no_voice_activity_chunks += 1
                if no_voice_activity_chunks > 3:
                    if not self.clients[websocket].eos:
                        self.clients[websocket].set_eos(True)
                    time.sleep(0.1)
                return no_voice_activity_chunks, False
            no_voice_activity_chunks = 0
            self.clients[websocket].set_eos(False)
            return no_voice_activity_chunks, True

        except Exception as e:
            logging.error(e)
            return no_voice_activity_chunks, False


    def disconnect_client(self, websocket):
        self.clients[websocket].disconnect()
        logging.warning(f"{self.clients[websocket]} Client disconnected due to overtime.")
        self.clients[websocket].cleanup()
        self.clients.pop(websocket)
        self.clients_start_time.pop(websocket)
        websocket.close()
        del websocket

    """Receives audio data from a websocket client.
    
    Performs voice activity detection, segments audio into frames, runs speech recognition
    with Whisper, and sends transcription results back to the client. Handles connecting 
    and disconnecting clients.
    """
    def recv_audio(self, websocket, transcription_queue=None, llm_queue=None, tts_playing_event=None):
        logging.info("[Transcription:] New client connected")

        self.vad_model = VoiceActivityDetection()
        self.vad_threshold = 0.5

        options = self.options_processing(websocket)
        self.handle_client_queue(websocket, options)

        device, compute = self.handle_transcriber()

        client = self.assign_client(websocket, options, transcription_queue, llm_queue)

        no_voice_activity_chunks = 0

        while True:
            if not tts_playing_event.is_set():
                frame_data, frame_np = self.frame_processing(websocket, client)
                no_voice_activity_chunks, continue_processing = self.voice_activity_detection(frame_np, no_voice_activity_chunks, websocket)
                if not continue_processing:
                    continue
                self.clients[websocket].add_frames(frame_np)

                elapsed_time = time.time() - self.clients_start_time[websocket]
                if elapsed_time >= self.max_connection_time:
                    self.disconnect_client(websocket)
                    break

    def run(self, host, port=9090, transcription_queue=None, llm_queue=None, tts_playing_event=None):
        """
        Run the transcription server.

        Args:
            host (str): The host address to bind the server.
            port (int): The port number to bind the server.
        """
        with serve(
            functools.partial(
                self.recv_audio,
                transcription_queue=transcription_queue,
                llm_queue=llm_queue,
                tts_playing_event=tts_playing_event
            ),
            host,
            port
        ) as server:
            server.serve_forever()




if __name__ == "__main__":
    from multiprocessing import Event
    tts_playing_event = Event()
    server = TranscriptionServer()
    server.run("0.0.0.0", 6006, None, None, tts_playing_event)