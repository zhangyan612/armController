#!/usr/bin/env python3

"""
Fastest TTS engine
"""
import asyncio
import time
import os
import vlc
import edge_tts


# refer to tts_make.launch   roslaunch tts tts_make.launch

def playSound(file):
    p = vlc.MediaPlayer(file)
    p.play()
    time.sleep(1)
    while p.is_playing():
        time.sleep(1)

def split_text_by_sentence(text):
    result = []  # List to store sentences
    sentence = ''  # Temporary variable to construct sentences
    char_count = 0  # Character counter

    for char in text:
        sentence += char
        char_count += 1

        # Check if the current character is a period and if we have enough characters to split
        if char == '.' and char_count >= 30:
            result.append(sentence.strip())  # Add the sentence to the result list
            sentence = ''  # Reset the temporary sentence variable
            char_count = 0  # Reset the character counter

    # Add the last sentence if there is any remaining text after the last period
    if sentence:
        result.append(sentence.strip())

    return result

# en-US-EricNeural
# en-US-GuyNeural
# en-US-RogerNeural
# en-US-SteffanNeural

async def generate_voice(text="Hello this is a test run", voice="en-US-SteffanNeural"):
    # Generate a timestamp for the output file name
    timestamp = time.strftime("%Y%m%d-%H%M%S")

    # Get the current working directory
    cwd = os.getcwd()
    output_file = os.path.join(cwd, f"files/voice/{timestamp}.mp3")
    print(output_file)
    communicate = edge_tts.Communicate(text, voice)
    with open(output_file, "wb") as file:
        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                file.write(chunk["data"])
            elif chunk["type"] == "WordBoundary":
                print(f"WordBoundary: {chunk}")

    # Play the generated audio file
    playSound(output_file)


if __name__ == "__main__":
    loop = asyncio.get_event_loop_policy().get_event_loop()
    try:
        loop.run_until_complete(generate_voice('This is a test run on generating voice. This is a long text that should be split into multiple files. Each file will be played seperately.'))
    finally:
        loop.close()

    # playSound('D:\Robot\SuperAGI/files/voice/20240124-220540.mp3')

