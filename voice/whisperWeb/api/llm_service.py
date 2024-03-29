
import time
import logging
logging.basicConfig(level = logging.INFO)
import time
from faster_whisper.utils import get_logger
import leptonLLM

class LLMService:
    def __init__(self):
        self.logger = get_logger()
        self.prompt_template = None
        self.last_prompt = None
        self.last_output = None
    
    def format_prompt_llm_api(self, prompt, conversation_history, system_prompt=""):
        messageList = []
        if conversation_history:
            messageList = conversation_history
            messageList.append({"role": "user", "content": prompt})
        else:
            messageList = [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": prompt},
            ]
        return messageList

    def run(
        self,
        # model_path,
        # tokenizer_path,
        transcription_queue=None,
        llm_queue=None,
        audio_queue=None,
        input_text=None, 
        max_output_len=50, 
        max_attention_window_size=4096, 
        num_beams=1, 
        streaming=False,
        streaming_interval=4,
        debug=False,
    ):        
        self.logger.info("[LLM Service]: Started")

        conversation_history = {}

        while True:

            # Get the last transcription output from the queue
            transcription_output = transcription_queue.get()
            if transcription_queue.qsize() != 0:
                continue
            
            if transcription_output["uid"] not in conversation_history:
                conversation_history[transcription_output["uid"]] = []

            prompt = transcription_output['prompt'].strip()

            logging.info(f'[LLM Service]: Prompt: {prompt}')

            # if prompt is same but EOS is True, we need that to send outputs to websockets
            # if self.last_prompt == prompt:
            #     if self.last_output is not None and transcription_output["eos"]:
            #         self.eos = transcription_output["eos"]
            #         logging.info('[LLM Service]:assign eos, not sending')          

                    # llm_queue.put({
                    #     "uid": transcription_output["uid"],
                    #     "llm_output": self.last_output,
                    #     "eos": self.eos,
                    #     "latency": self.infer_time
                    # })
                    # audio_queue.put({"llm_output": self.last_output, "eos": self.eos})
                    # conversation_history[transcription_output["uid"]].append(
                    #     (transcription_output['prompt'].strip(), self.last_output[0].strip())
                    # )
                    # continue

            system_prompt = "You are a Robot, a helpful AI assistant, here is a list of conversation history, you will generate answer based on the history"

            input_prompt = self.format_prompt_llm_api(prompt, conversation_history[transcription_output["uid"]], system_prompt=system_prompt)
            
            self.eos = transcription_output["eos"]
            logging.info(input_prompt)

            if self.eos and not prompt == '':
                should_ask_llm = False
                weakup_words = ['hi robot', 'hey robot']
                for word in weakup_words:
                    if word in prompt.lower():
                        should_ask_llm = True

                if should_ask_llm:
                    logging.info(f"[LLM Service]: Send to LLM prompt: {prompt}, eos: {self.eos}")

                    start = time.time()
                    output = leptonLLM.llm_request(input_prompt)
                    self.infer_time = time.time() - start
                    
                    # if self.eos:
                    if output is not None:
                        output = clean_llm_output(output)

                        self.last_output = output
                        self.last_prompt = prompt
                        llm_queue.put({
                            "uid": transcription_output["uid"],
                            "llm_output": output,
                            "eos": self.eos,
                            "latency": self.infer_time
                        })
                        audio_queue.put({"llm_output": output, "eos": self.eos})
                        logging.info(f"[LLM Service]: Output sent to llm_queue: {output}, inference done in {self.infer_time} ms\n")

                        conversation_history[transcription_output["uid"]].append(
                            (transcription_output['prompt'].strip(), output.strip())
                        )
                        self.last_prompt = None
                        self.last_output = None


def clean_llm_output(output):
    output = output.replace("\n\nDolphin\n\n", "")
    output = output.replace("\nDolphin\n\n", "")
    output = output.replace("Dolphin: ", "")
    output = output.replace("Assistant: ", "")

    if not output.endswith('.') and not output.endswith('?') and not output.endswith('!'):
        last_punct = output.rfind('.')
        last_q = output.rfind('?')
        if last_q > last_punct:
            last_punct = last_q
        
        last_ex = output.rfind('!')
        if last_ex > last_punct:
            last_punct = last_ex
        
        if last_punct > 0:
            output = output[:last_punct+1]

    return output



# if __name__ == "__main__":
#     multiprocessing.set_start_method('spawn')
    
#     lock = multiprocessing.Lock()
    
#     manager = Manager()
#     shared_output = manager.list()

#     transcription_queue = Queue()
#     llm_queue = Queue()
#     audio_queue = Queue()


#     llm_provider = EmptyLLM()
#     llm_process = multiprocessing.Process(
#         target=llm_provider.run,
#         args=(
#             transcription_queue,
#             llm_queue,
#             audio_queue,
#         )
#     )

#     llm_process.start()
