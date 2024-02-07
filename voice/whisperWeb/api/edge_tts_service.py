import functools
import time
import logging
import asyncio
import os
import edge_tts
import vlc 

logging.basicConfig(level = logging.INFO)
from websockets.sync.server import serve

class EdgeTTSService:
    def __init__(self):
        pass
    
    def initialize_model(self):
        self.last_llm_response = None

    def run(self, host, port, audio_queue=None, tts_playing_event=None):
        self.initialize_model()
        logging.info("[TTS Service]: Started")

        with serve(
            functools.partial(self.start_edge_tts, audio_queue=audio_queue, tts_playing_event=tts_playing_event), 
            host, port
            ) as server:
            server.serve_forever()


    # async def generate_voice(self, text="Hello this is a test run", voice="en-US-SteffanNeural"):
    #     communicate = edge_tts.Communicate(text, voice)
    #     audio_data = b""
    #     async for chunk in communicate.stream():
    #         if chunk["type"] == "audio":
    #             audio_data += chunk["data"]
    #     return audio_data

    # def start_edge_tts(self, websocket, audio_queue=None):
    #     self.eos = False
    #     self.output_audio = None

    #     while True:
    #         llm_response = audio_queue.get()
    #         if audio_queue.qsize() != 0:
    #             continue
            
    #         if isinstance(llm_response, str):
    #             logging.info("[TTS Service]: LLM Response received from tts service:" + llm_response)
    #         try:
    #             websocket.ping()
    #         except Exception as e:
    #             del websocket
    #             audio_queue.put(llm_response)
    #             logging.error(f"[TTS ERROR]: put llm response to audio queue")
    #             break
            
    #         llm_output = llm_response["llm_output"]
    #         self.eos = llm_response["eos"]

    #         def should_abort():
    #             if not audio_queue.empty(): raise TimeoutError()

    #         # only process if the output updated
    #         if self.last_llm_response != llm_output.strip():
    #             try:
    #                 start = time.time()
    #                 new_loop = asyncio.new_event_loop()
    #                 asyncio.set_event_loop(new_loop)
    #                 audio_bytes = new_loop.run_until_complete(self.generate_voice(llm_output.strip()))
    #                 # print(audio_bytes)
    #                 self.output_audio = audio_bytes
    #                 self.last_llm_response = llm_output.strip()
    #                 inference_time = time.time() - start
    #             except TimeoutError:
    #                 pass
    #             finally:
    #                 new_loop.close()

    #             logging.info(f"[TTS Service]: TTS inference done in {inference_time} ms.")

    #         if self.eos and self.output_audio is not None:
    #             try:
    #                 websocket.send(self.output_audio)
    #                 logging.info(f"[TTS Service]: Sent audio to websocket")
    #             except Exception as e:
    #                 logging.error(f"[TTS ERROR]: Audio error: {e}")
    def playSound(self, file):
        p = vlc.MediaPlayer(file)
        p.play()
        time.sleep(1)
        while p.is_playing():
            time.sleep(1)

    def start_edge_tts(self, websocket, audio_queue=None, tts_playing_event=None):
        self.eos = False
        self.output_audio = None

        while True:
            llm_response = audio_queue.get()
            if audio_queue.qsize() != 0:
                continue
            
            # if isinstance(llm_response, str):
            #     logging.info("[TTS Service]: LLM Response received from tts service:" + llm_response)
            # try:
            #     websocket.ping()
            # except Exception as e:
            #     del websocket
            #     audio_queue.put(llm_response)
            #     logging.error(f"[TTS ERROR]: put llm response to audio queue")
            #     break
            
            llm_output = llm_response["llm_output"]
            self.eos = llm_response["eos"]

            # def should_abort():
            #     if not audio_queue.empty(): raise TimeoutError()

            # only process if the output updated
            if self.last_llm_response != llm_output.strip():
                try:
                    start = time.time()
                    new_loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(new_loop)
                    audio_file_path = new_loop.run_until_complete(self.generate_voice(llm_output.strip()))
                    inference_time = time.time() - start
                    logging.info(f"[TTS Service]: TTS inference done in {inference_time} ms.")
                    self.last_llm_response = llm_output.strip()
                    self.output_audio = audio_file_path
                except TimeoutError:
                    pass
                
            if self.eos and self.output_audio is not None:
                try:
                    # with open(audio_file_path, 'rb') as f:
                    #     self.output_audio = f.read()
                    # websocket.send(self.output_audio)
                    tts_playing_event.set()

                    self.playSound(self.output_audio)
                    logging.info(f"[TTS Service]: playing audio file")
                    if os.path.isfile(self.output_audio):
                        os.remove(self.output_audio)
                        self.output_audio = None

                    tts_playing_event.clear()

                except Exception as e:
                    logging.error(f"[TTS ERROR]: Audio error: {e}")

    async def generate_voice(self, text="Hello this is a test run", voice="en-US-SteffanNeural"):
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        cwd = os.getcwd()
        output_file = os.path.join(cwd, f"{timestamp}.wav")
        # print(output_file)
        communicate = edge_tts.Communicate(text, voice)
        with open(output_file, "wb") as file:
            async for chunk in communicate.stream():
                if chunk["type"] == "audio":
                    file.write(chunk["data"])
                elif chunk["type"] == "WordBoundary":
                    print(f"WordBoundary: {chunk}")

        return output_file

