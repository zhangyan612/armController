#!/usr/bin/env python3

import asyncio
import edge_tts

async def generate_voice(text="Hello this is a test run", voice="en-US-SteffanNeural"):
    communicate = edge_tts.Communicate(text, voice)
    audio_data = b""
    async for chunk in communicate.stream():
        if chunk["type"] == "audio":
            audio_data += chunk["data"]
        elif chunk["type"] == "WordBoundary":
            print(f"WordBoundary: {chunk}")
    return audio_data

if __name__ == "__main__":
    text = 'this is test file'
    loop = asyncio.get_event_loop_policy().get_event_loop()
    try:
        audio_bytes = loop.run_until_complete(generate_voice(text))
        print(audio_bytes)
    finally:
        loop.close()



