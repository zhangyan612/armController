import time

class leptonLLM:
    def __init__(self) -> None:
        self.counter = 0

    def llm_request(self, messageList, model='mixtral-8x7b'):
        time.sleep(5)
        print(messageList)
        text = f'This is output {str(self.counter)} from AI, and it should be a long text that could block transcriber service'
        self.counter= self.counter+1
        return text

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

