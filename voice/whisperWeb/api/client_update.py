import os
import wave

import numpy as np
# import scipy
# import ffmpeg
import pyaudio
import threading
import textwrap
# import json
# import websocket
import uuid
import time
from faster_whisper import WhisperModel
import torch
from vad import VoiceActivityDetection

model_size = "base.en"
os.environ['KMP_DUPLICATE_LIB_OK']='True'

class UpdatedClient:
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

        # self.stream = self.p.open(format=FORMAT,
        #         channels=CHANNELS,
        #         rate=self.RATE,
        #         input=True,
        #         frames_per_buffer=CHUNK)

        # self.model = WhisperModel(model_size, device="cpu", compute_type="int8")
        device = "cpu"
        compute = "int8"
        self.transcriber = WhisperModel(
            model_size_or_path="base.en",
            device=device,
            compute_type=compute,
            local_files_only=False,
        )

        self.client = ServeClient(
            # websocket,
            multilingual=False,
            language='en',
            # task=options["task"],
            # client_uid=options["uid"],
            # transcription_queue=transcription_queue,
            # llm_queue=llm_queue,
            transcriber=self.transcriber
        )

        UpdatedClient.INSTANCES[self.uid] = self

        self.frames = b""
        print("[INFO]: * recording")


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
            print('send message to transcribe')
            # print(message)
            # frame_np = np.frombuffer(message, dtype=np.float32)
            # self.write_output_recording(n_audio_file, message)

            segments_all, info = self.model.transcribe(message, beam_size=5)
            for segment in segments_all:
                print("[%.2fs -> %.2fs] %s" % (segment.start, segment.end, segment.text))


            # self.client_socket.send(message, websocket.ABNF.OPCODE_BINARY)
        except Exception as e:
            print(e)

    def write_audio_frames_to_file(self, frames, file_name, rate=None):
        """
        Write audio frames to a WAV file.

        The WAV file is created or overwritten with the specified name. The audio frames should be 
        in the correct format and match the specified channel, sample width, and sample rate.

        Args:
            frames (bytes): The audio frames to be written to the file.
            file_name (str): The name of the WAV file to which the frames will be written.

        """
        with wave.open(file_name, "wb") as wavfile:
            wavfile: wave.Wave_write
            wavfile.setnchannels(self.channels)
            wavfile.setsampwidth(2)
            wavfile.setframerate(self.rate if rate is None else rate)
            wavfile.writeframes(frames)

    def voice_activity_detection(self, frame_np, no_voice_activity_chunks):
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
            speech_prob = self.vad_model(torch.from_numpy(frame_np.copy()), self.rate).item()
            if speech_prob < self.vad_threshold:
                no_voice_activity_chunks += 1
                if no_voice_activity_chunks > 3:
                    if not self.client.eos:
                        self.client.set_eos(True)
                    time.sleep(0.1)
                return no_voice_activity_chunks, False
            no_voice_activity_chunks = 0
            self.client.set_eos(False)
            return no_voice_activity_chunks, True
        except Exception as e:
            print(e)
            return no_voice_activity_chunks, False


    def record(self):

        print('recording')
        n_audio_file = 0
        if not os.path.exists("chunks"):
            os.makedirs("chunks", exist_ok=True)

        while True:
            # if not self.recording:
            #     break
            data = self.stream.read(self.chunk)
            self.frames += data

            audio_array = UpdatedClient.bytes_to_float_array(self.frames)
            # raw_data = np.frombuffer(buffer=self.frames, dtype=np.int16)
            duration = audio_array.shape[0] / self.rate
            print(duration)
            if duration < 1:
                continue
            # self.send_packet_to_server(audio_array)
            print('save to file')
            t = threading.Thread(
                target=self.write_audio_frames_to_file,
                args=(
                    self.frames[:],
                    f"chunks/{n_audio_file}.wav",
                ),
            )
            t.start()

            # save frames if more than a minute
            if len(self.frames) > 60 * self.rate:
                n_audio_file += 1
                self.frames = b""


    def record_client(self):

        print('recording')
        n_audio_file = 0
        # if not os.path.exists("chunks"):
        #     os.makedirs("chunks", exist_ok=True)
        self.vad_model = VoiceActivityDetection()
        self.vad_threshold = 0.5
        no_voice_activity_chunks = 0

        while True:

            # frame_data = websocket.recv()  # change get data from local mic
            data = self.stream.read(self.chunk)
            self.frames += data

            # audio_array = UpdatedClient.bytes_to_float_array(self.frames)
            audio_array = UpdatedClient.bytes_to_float_array(data)

            # self.send_packet_to_server(audio_array.tobytes())
            # frame_data = websocket.recv()  # change get data from local mic

            frame_np = np.frombuffer(audio_array.tobytes(), dtype=np.float32)

            no_voice_activity_chunks, continue_processing = self.voice_activity_detection(frame_np, no_voice_activity_chunks)
            if not continue_processing:
                continue
            print('add frame')
            self.client.add_frames(frame_np)

            # elapsed_time = time.time() - self.clients_start_time
            # if elapsed_time >= self.max_connection_time:
            #     # self.disconnect_client(websocket)
            #     break
            # if len(self.frames) > 60 * self.rate:
            #     n_audio_file += 1
            #     self.frames = b""


    def write_output_recording(self, n_audio_file, frames, out_file="output_recording.wav"):
            """
            Combine and save recorded audio chunks into a single WAV file.
            
            The individual audio chunk files are expected to be located in the "chunks" directory. Reads each chunk 
            file, appends its audio data to the final recording, and then deletes the chunk file. After combining
            and saving, the final recording is stored in the specified `out_file`.


            Args:
                n_audio_file (int): The number of audio chunk files to combine.
                out_file (str): The name of the output WAV file to save the final recording.

            """
            # input_files = [
            #     f"chunks/{i}.wav"
            #     for i in range(n_audio_file)
            #     if os.path.exists(f"chunks/{i}.wav")
            # ]
            with wave.open(out_file, "wb") as wavfile:
                wavfile: wave.Wave_write
                wavfile.setnchannels(self.channels)
                wavfile.setsampwidth(2)
                wavfile.setframerate(self.rate)
                # data = wav_in.readframes(frames)
                # if data == b"":
                #     break
                wavfile.writeframes(n_audio_file)

                # for in_file in input_files:
                #     with wave.open(in_file, "rb") as wav_in:
                #         while True:
                #             data = wav_in.readframes(frames)
                #             if data == b"":
                #                 break
                #             wavfile.writeframes(data)
                    # remove this file
                    # os.remove(in_file)
            wavfile.close()

            
class TranscriptionClient:

    def __init__(self, host, port, is_multilingual=False, lang=None, translate=False, model_size="small"):
        self.client = UpdatedClient(host, port, is_multilingual, lang, translate, model_size)

    def __call__(self, audio=None, hls_url=None):

        print("[INFO]: Waiting for server ready ...")

        self.recording = True
        self.client.record_client()



class ServeClient:
    RATE = 16000
    SERVER_READY = "SERVER_READY"
    DISCONNECT = "DISCONNECT"

    def __init__(
        self,
        # websocket,
        task="transcribe",
        device=None,
        multilingual=False,
        language=None, 
        client_uid=None,
        # transcription_queue=None,
        # llm_queue=None,
        transcriber=None
        ):

        if transcriber is None:
            raise ValueError("Transcriber is None.")
        self.transcriber = transcriber
        self.client_uid = client_uid
        # self.transcription_queue = transcription_queue
        # self.llm_queue = llm_queue
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
        # self.websocket = websocket
        self.lock = threading.Lock()
        self.eos = False
        self.trans_thread = threading.Thread(target=self.speech_to_text)
        self.trans_thread.start()

    def set_eos(self, eos):
        self.lock.acquire()
        self.eos = eos
        self.lock.release()

    def add_frames(self, frame_np):
        """
        Add audio frames to the ongoing audio stream buffer.

        This method is responsible for maintaining the audio stream buffer, allowing the continuous addition
        of audio frames as they are received. It also ensures that the buffer does not exceed a specified size
        to prevent excessive memory usage.

        If the buffer size exceeds a threshold (45 seconds of audio data), it discards the oldest 30 seconds
        of audio data to maintain a reasonable buffer size. If the buffer is empty, it initializes it with the provided
        audio frame. The audio stream buffer is used for real-time processing of audio data for transcription.

        Args:
            frame_np (numpy.ndarray): The audio frame data as a NumPy array.

        """
        self.lock.acquire()

        if self.frames_np is not None and self.frames_np.shape[0] > 45*self.RATE:
            self.frames_offset += 30.0
            self.frames_np = self.frames_np[int(30*self.RATE):]
        if self.frames_np is None:
            self.frames_np = frame_np.copy()
        else:
            self.frames_np = np.concatenate((self.frames_np, frame_np), axis=0)
        self.lock.release()

    def update_prompt_and_segments(self, result, duration):
        if len(result):
            self.t_start = None
            last_segment = self.update_segments(result, duration)
            if len(self.transcript) < self.send_last_n_segments:
                segments = self.transcript
            else:
                segments = self.transcript[-self.send_last_n_segments:]
            if last_segment is not None:
                segments = segments + [last_segment]                    
        else:
            # show previous output if there is pause i.e. no output from whisper
            segments = []
            if self.t_start is None: self.t_start = time.time()
            if time.time() - self.t_start < self.show_prev_out_thresh:
                if len(self.transcript) < self.send_last_n_segments:
                    segments = self.transcript
                else:
                    segments = self.transcript[-self.send_last_n_segments:]
            
            # add a blank if there is no speech for 3 seconds
            if len(self.text) and self.text[-1] != '':
                if time.time() - self.t_start > self.add_pause_thresh:
                    self.text.append('')
        return segments

    def update_ui_and_queue(self, segments, infer_time, duration):
        try:
            self.prompt = ' '.join(segment['text'] for segment in segments)

            print(f"[Transcription]: Send segments to web socket")

            # self.transcription_queue.put({"uid": self.client_uid, "prompt": self.prompt, "eos": self.eos})
            print(f"[Transcription]: Send message to transcription queue {self.prompt}")
            if self.eos:
                    self.timestamp_offset += duration
                    print(f"[Transcription]: EOS: {self.eos} Prompt: {self.prompt}")
                    print(
                        f"[Transcription]: Average inference time {sum(self.segment_inference_time) / len(self.segment_inference_time)}\n")
                    self.segment_inference_time = []

        except Exception as e:
            print(f"[Transcription socket ERROR]: {e}")


    def speech_to_text(self):
        """
        Transcribes audio to text using Whisper in a loop. 
        
        Processes incoming audio in chunks, runs Whisper inference, 
        updates transcript with new segments, sends updates to client.
        
        Manages state like current prompt, transcript, timestamps, etc.
        to enable real-time streaming transcription.
        """
        while True:
            # send the LLM outputs
            # self.check_llm_queue()

            if self.exit:
                print("[Transcription]: Exiting speech to text thread")
                break
            
            if self.frames_np is None: 
                time.sleep(0.02)    # wait for any audio to arrive
                continue

            # clip audio if the current chunk exceeds 30 seconds, this basically implies that
            # no valid segment for the last 30 seconds from whisper
            if self.frames_np[int((self.timestamp_offset - self.frames_offset)*self.RATE):].shape[0] > 25 * self.RATE:
                duration = self.frames_np.shape[0] / self.RATE
                self.timestamp_offset = self.frames_offset + duration - 5

            samples_take = max(0, (self.timestamp_offset - self.frames_offset)*self.RATE)
            input_bytes = self.frames_np[int(samples_take):].copy()
            duration = input_bytes.shape[0] / self.RATE

            if duration < self.sleep_duration:
                time.sleep(0.01)    # 5ms sleep to wait for some voice active audio to arrive
                continue
            try:
                input_sample = input_bytes.copy()
                start = time.time()

                # whisper transcribe with prompt
                result, info = self.transcriber.transcribe(
                    input_sample, 
                    initial_prompt=None,
                    language=self.language,
                    task=self.task,
                    vad_filter=True,
                    vad_parameters={"threshold": 0.5}
                )
                print(info)
                
                infer_time = time.time() - start
                self.segment_inference_time.append(infer_time)
                
                segments = self.update_prompt_and_segments(result, duration)

                self.update_ui_and_queue(segments, infer_time, duration)

            except Exception as e:
                print(f"[Transcription ERROR]: {e}")
                time.sleep(0.01)
    
    def update_segments(self, segments, duration):
        offset = None
        self.current_out = ''
        last_segment = None
        # process complete segments
        if len(segments) > 1:
            for i, s in enumerate(segments[:-1]):
                text_ = s.text
                self.text.append(text_)
                start, end = self.timestamp_offset + s.start, self.timestamp_offset + min(duration, s.end)
                self.transcript.append(
                    {
                        'start': start,
                        'end': end,
                        'text': text_
                    }
                )
                
                offset = min(duration, s.end)

        self.current_out += segments[-1].text
        last_segment = {
            'start': self.timestamp_offset + segments[-1].start,
            'end': self.timestamp_offset + min(duration, segments[-1].end),
            'text': self.current_out
        }
        
        # if same incomplete segment is seen multiple times then update the offset
        # and append the segment to the list
        if self.current_out.strip() == self.prev_out.strip() and self.current_out != '': 
            self.same_output_threshold += 1
        else: 
            self.same_output_threshold = 0
        
        if self.same_output_threshold > 5:
            if not len(self.text) or self.text[-1].strip().lower()!=self.current_out.strip().lower():          
                self.text.append(self.current_out)
                self.transcript.append(
                    {
                        'start': self.timestamp_offset,
                        'end': self.timestamp_offset + duration,
                        'text': self.current_out
                    }
                )
            self.current_out = ''
            offset = duration
            self.same_output_threshold = 0
            last_segment = None
        else:
            self.prev_out = self.current_out
        
        # update offset
        if offset is not None:
            self.timestamp_offset += offset

        return last_segment

    def cleanup(self):
        self.exit = True


if __name__ == "__main__":
    client = TranscriptionClient(
    "localhost",
    6006,
    is_multilingual=False,
    lang="en",
    translate=False,
    )
    client()



# File object has no read() method, or readable() returned False.