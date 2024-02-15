

class LLMMemory:
    def __init__(self, systemPrompt='') -> None:
        self.messageList = [
            {"role": "system", "content": systemPrompt},
        ]

    def add_message_history(self, role, message):
        self.messageList.append({"role": role, "content": message})
        # if message exceed token limit, use a sliding window to remove
        return self.messageList
