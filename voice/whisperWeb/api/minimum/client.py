import os
import wave
import numpy as np
import scipy
import ffmpeg
import pyaudio
import threading
import textwrap
import json
import websocket
import uuid
import time
import webrtcvad


class Client:
    INSTANCES = {}

    def __init__(
        self, host=None, port=None, is_multilingual=False, lang=None, translate=False, model_size="base.en"
    ):
        self.chunk = 1024 * 3
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000
        self.record_seconds = 60000
        self.recording = False
        self.multilingual = False
        self.language = None
        self.task = "transcribe"
        self.uid = str(uuid.uuid4())
        self.waiting = False
        self.last_response_recieved = None
        self.disconnect_if_no_response_for = 15
        self.multilingual = is_multilingual
        self.language = lang
        self.model_size = model_size
        self.server_error = False
        if translate:
            self.task = "translate"

        self.timestamp_offset = 0.0
        self.audio_bytes = None
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk,
        )
        self.webrtc_vad_model = webrtcvad.Vad()
        self.webrtc_vad_model.set_mode(1)  # set aggressiveness mode, in [0, 3]

        if host is not None and port is not None:
            socket_url = f"ws://{host}:{port}"
            self.client_socket = websocket.WebSocketApp(
                socket_url,
                on_open=lambda ws: self.on_open(ws),
                on_message=lambda ws, message: self.on_message(ws, message),
                on_error=lambda ws, error: self.on_error(ws, error),
                on_close=lambda ws, close_status_code, close_msg: self.on_close(
                    ws, close_status_code, close_msg
                ),
            )
        else:
            print("[ERROR]: No host or port specified.")
            return

        Client.INSTANCES[self.uid] = self

        # start websocket client in a thread
        self.ws_thread = threading.Thread(target=self.client_socket.run_forever)
        self.ws_thread.setDaemon(True)
        self.ws_thread.start()

        self.frames = b""
        print("[INFO]: * recording")


    def on_message(self, ws, message):
        self.last_response_recieved = time.time()
        message = json.loads(message)

        if self.uid != message.get("uid"):
            print("[ERROR]: invalid client uid")
            return

        if "status" in message.keys():
            if message["status"] == "WAIT":
                self.waiting = True
                print(
                    f"[INFO]:Server is full. Estimated wait time {round(message['message'])} minutes."
                )
            elif message["status"] == "ERROR":
                print(f"Message from Server: {message['message']}")
                self.server_error = True
            return

        if "message" in message.keys() and message["message"] == "DISCONNECT":
            print("[INFO]: Server overtime disconnected.")
            self.recording = False

        if "message" in message.keys() and message["message"] == "SERVER_READY":
            self.recording = True
            return

        if "language" in message.keys():
            self.language = message.get("language")
            lang_prob = message.get("language_prob")
            print(
                f"[INFO]: Server detected language {self.language} with probability {lang_prob}"
            )
            return

        if "llm_output" in message.keys():
            print("LLM output: ")
            for item in message["llm_output"]:
                print(item)


        if "segments" not in message.keys():
            return

        message = message["segments"]
        text = []
        print(message)
        if len(message):
            for seg in message:
                if text and text[-1] == seg["text"]:
                    # already got it
                    continue
                text.append(seg["text"])
        # keep only last 3
        if len(text) > 3:
            text = text[-3:]
        wrapper = textwrap.TextWrapper(width=60)
        word_list = wrapper.wrap(text="".join(text))
        # Print each line.
        # if os.name == "nt":
        #     os.system("cls")
        # else:
        #     os.system("clear")
        for element in word_list:
            print(element)

    def on_error(self, ws, error):
        print(error)

    def on_close(self, ws, close_status_code, close_msg):
        print(f"[INFO]: Websocket connection closed: {close_status_code}: {close_msg}")

    def on_open(self, ws):
        """
        Callback function called when the WebSocket connection is successfully opened.
        
        Sends an initial configuration message to the server, including client UID, multilingual mode,
        language selection, and task type.

        Args:
            ws (websocket.WebSocketApp): The WebSocket client instance.

        """
        print(self.multilingual, self.language, self.task)

        print("[INFO]: Opened connection")
        ws.send(
            json.dumps(
                {
                    "uid": self.uid,
                    "multilingual": self.multilingual,
                    "language": self.language,
                    "task": self.task,
                    "model_size": self.model_size,
                }
            )
        )
    
    
    @staticmethod
    def bytes_to_float_array(audio_bytes):
        """
        Convert audio data from bytes to a NumPy float array.
        
        It assumes that the audio data is in 16-bit PCM format. The audio data is normalized to 
        have values between -1 and 1.

        Args:
            audio_bytes (bytes): Audio data in bytes.

        Returns:
            np.ndarray: A NumPy array containing the audio data as float values normalized between -1 and 1.
        """
        raw_data = np.frombuffer(buffer=audio_bytes, dtype=np.int16)
        return raw_data.astype(np.float32) / 32768.0

    def send_packet_to_server(self, message):
        """
        Send an audio packet to the server using WebSocket.

        Args:
            message (bytes): The audio data packet in bytes to be sent to the server.

        """
        try:
            print('send msg to server')
            self.client_socket.send(message, websocket.ABNF.OPCODE_BINARY)
        except Exception as e:
            print(e)

    def close_websocket(self):
        try:
            self.client_socket.close()
        except Exception as e:
            print("[ERROR]: Error closing WebSocket:", e)

        try:
            self.ws_thread.join()
        except Exception as e:
            print("[ERROR:] Error joining WebSocket thread:", e)

    def _is_webrtc_speech(self, data, all_frames_must_be_true=False):
        """
        Returns true if speech is detected in the provided audio data

        Args:
            data (bytes): raw bytes of audio data (1024 raw bytes with
            16000 sample rate and 16 bits per sample)
        """
        sample_rate = self.rate
        # Number of audio frames per millisecond
        frame_length = int(sample_rate * 0.01)  # for 10ms frame
        num_frames = int(len(data) / (2 * frame_length))
        speech_frames = 0

        for i in range(num_frames):
            start_byte = i * frame_length * 2
            end_byte = start_byte + frame_length * 2
            frame = data[start_byte:end_byte]
            if self.webrtc_vad_model.is_speech(frame, sample_rate):
                speech_frames += 1
                if not all_frames_must_be_true:
                    return True
        if all_frames_must_be_true:
            return speech_frames == num_frames
        else:
            return False

    def record(self, out_file="output_recording.wav"):
        n_audio_file = 0
        try:
            for _ in range(0, int(self.rate / self.chunk * self.record_seconds)):
                if not self.recording:
                    break
                data = self.stream.read(self.chunk)
                self.frames += data
                voice_detected = self._is_webrtc_speech(data)
                if voice_detected:
                    print('send voice')
                    audio_array = Client.bytes_to_float_array(data)
                    self.send_packet_to_server(audio_array.tobytes())

                # save frames if more than a minute
                if len(self.frames) > 60 * self.rate:
                    # n_audio_file += 1
                    self.frames = b""

        except KeyboardInterrupt:
            self.stream.stop_stream()
            self.stream.close()
            self.p.terminate()
            self.close_websocket()

class TranscriptionClient:
    def __init__(self, host, port, is_multilingual=False, lang=None, translate=False, model_size="base.en"):
        self.client = Client(host, port, is_multilingual, lang, translate, model_size)

    def __call__(self, audio=None, hls_url=None):
        print("[INFO]: Waiting for server ready ...")
        while not self.client.recording:
            if self.client.waiting or self.client.server_error:
                self.client.close_websocket()
                return

        print("[INFO]: Server Ready!")
        self.client.record()


if __name__ == "__main__":
    client = TranscriptionClient(
    "localhost",
    6006,
    is_multilingual=False,
    lang="en",
    translate=False,
    )
    client()
