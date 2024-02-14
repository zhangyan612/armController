import time
import json
from websockets.sync.server import serve
from vad import VoiceActivityDetection
import torch
import numpy as np
import time
from transcriber import WhisperModel
import os
import functools
import platform
import threading
import json
import textwrap
import queue
import wave

import logging
logging.basicConfig(level = logging.INFO)

os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"

class TranscriptionServer:

    def __init__(self):
        self.RATE = 16000
        self.clients = {}
        self.websockets = {}
        self.clients_start_time = {}
        self.max_clients = 4
        self.max_connection_time = 600
        self.transcriber = None

    def get_wait_time(self):
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
                # frame_data = websocket.recv()
                # frame_np = np.frombuffer(frame_data, dtype=np.float32)
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



class ServeClient:
    RATE = 16000
    SERVER_READY = "SERVER_READY"
    DISCONNECT = "DISCONNECT"

    def __init__(
        self,
        websocket,
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
        self.websocket = websocket
        self.lock = threading.Lock()
        self.eos = False
        self.trans_thread = threading.Thread(target=self.speech_to_text)
        self.trans_thread.start()
        # self.websocket.send(
        #     json.dumps(
        #         {
        #             "uid": self.client_uid,
        #             "message": self.SERVER_READY
        #         }
        #     )
        # )

    def set_eos(self, eos):
        self.lock.acquire()
        self.eos = eos
        self.lock.release()

    def fill_output(self, output):
        text = ''
        pick_prev = min(len(self.text), self.pick_previous_segments)
        for seg in self.text[-pick_prev:]:
            # discard everything before a 3 second pause
            if seg == '':
                text = ''
            else:
                text += seg
        wrapped = "".join(text + output)
        return wrapped
    
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

    # def check_llm_queue(self):
    #     try:
    #         llm_response = None
    #         if self.llm_queue is not None:
    #             while not self.llm_queue.empty():
    #                 llm_response = self.llm_queue.get()
                
    #             if llm_response:
    #                 eos = llm_response["eos"]
    #                 if eos:
    #                     logging.info(f"[Transcription]: Sending LLM response to web socket")
    #                     self.websocket.send(json.dumps(llm_response))
    #     except queue.Empty:
    #         pass

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

    # def check_language(self, info):
    #     if self.language is None:
    #         if info.language_probability > 0.5:
    #             self.language = info.language
    #             logging.info(f"Detected language {self.language} with probability {info.language_probability}")
    #             self.websocket.send(json.dumps(
    #                 {"uid": self.client_uid, "language": self.language, "language_prob": info.language_probability}))
    #         else:
    #             # detect language again
    #             logging.info("language_probability low, detect again")
    #             # continue

    def update_ui_and_queue(self, segments, infer_time, duration):
        try:
            self.prompt = ' '.join(segment['text'] for segment in segments)

            logging.info(f"[Transcription]: Send segments to web socket")
            self.websocket.send(
                json.dumps({
                    "uid": self.client_uid,
                    "segments": segments,
                    "eos": self.eos,
                    "latency": infer_time
                })
            )
            # self.transcription_queue.put({"uid": self.client_uid, "prompt": self.prompt, "eos": self.eos})
            logging.info(f"[Transcription]: Send message to transcription queue {self.prompt}")
            if self.eos:
                    self.timestamp_offset += duration
                    logging.info(f"[Transcription]: EOS: {self.eos} Prompt: {self.prompt}")
                    logging.info(
                        f"[Transcription]: Average inference time {sum(self.segment_inference_time) / len(self.segment_inference_time)}\n")
                    self.segment_inference_time = []

        except Exception as e:
            logging.error(f"[Transcription socket ERROR]: {e}")

    def write_audio_frames_to_file(self, frames, file_name, rate=None):
        channel = 1
        rate = 160000
        with wave.open(file_name, "wb") as wavfile:
            wavfile: wave.Wave_write
            wavfile.setnchannels(channel)
            wavfile.setsampwidth(2)
            wavfile.setframerate(rate)
            wavfile.writeframes(frames)

    def speech_to_text(self):
        n_audio_file = 0

        while True:
            # send the LLM outputs
            # self.check_llm_queue()

            if self.exit:
                logging.info("[Transcription]: Exiting speech to text thread")
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

                t = threading.Thread(
                    target=self.write_audio_frames_to_file,
                    args=(
                        input_sample.tobytes(),
                        f"chunks/{n_audio_file}.wav",
                    ),
                )
                t.start()

                # last_segment = self.update_segments(result, duration)
                # TODO: Check speaker of the file
                # I think the term is “speaker diarization”, and I see some guides online using “pyannote.audio” together with Whisper to achieve this - pyannote-whisper
                # logging.info(result)
                # logging.info(info)

                infer_time = time.time() - start
                self.segment_inference_time.append(infer_time)
                # check_language(info)
                
                segments = self.update_prompt_and_segments(result, duration)
                # segments = []
                # if len(last_segment):
                #     segments.append({"text": last_segment})

                self.update_ui_and_queue(segments, infer_time, duration)

            except Exception as e:
                logging.error(f"[Transcription ERROR]: {e}")
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
    
    def disconnect(self):
        self.websocket.send(
            json.dumps(
                {
                    "uid": self.client_uid,
                    "message": self.DISCONNECT
                }
            )
        )
    
    def cleanup(self):
        logging.info("Cleaning up.")
        self.exit = True


if __name__ == "__main__":
    from multiprocessing import Event
    tts_playing_event = Event()
    server = TranscriptionServer()
    # 'NoneType' object has no attribute 'put'  -- need transcription queue
    server.run("0.0.0.0", 6006, None, None, tts_playing_event)