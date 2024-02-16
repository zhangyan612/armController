import json

import httpx
import uuid
import openai
import datetime
import configparser

class leptonLLM:
    def __init__(self) -> None:
        config = configparser.ConfigParser()
        config.read('.env')
        self.LEPTON_WORKSPACE_TOKEN = config.get('LEPTON', 'token')

        self.stop_words = [
            "<|im_end|>",
            "[End]",
            "[end]",
            "\nReferences:\n",
            "\nSources:\n",
            "End.",
        ]

    def llm_request(self, messageList, model='mixtral-8x7b'):
        client = openai.OpenAI(
                base_url=f"https://{model}.lepton.run/api/v1/",
                api_key=self.LEPTON_WORKSPACE_TOKEN,
                timeout=httpx.Timeout(connect=10, read=120, write=120, pool=10),
            )
        
        llm_response = client.chat.completions.create(
            model=model,
            messages=messageList,
            max_tokens=1024,
            stop=self.stop_words,
            stream=False,
            temperature=0.9,
        )
        return llm_response.choices[0].message.content
        
if __name__ == '__main__':
    userPrompt = 'Whats the SOTA text to speech model?'
    systemPrompt = ''
    messageList = [
        {"role": "system", "content": systemPrompt},
        {"role": "user", "content": userPrompt},
    ]
    llm = leptonLLM()
    response = llm.llm_request(messageList)
    print(response)

