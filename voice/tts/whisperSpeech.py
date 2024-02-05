from whisperspeech.pipeline import Pipeline

pipe = Pipeline(s2a_ref='collabora/whisperspeech:s2a-q4-tiny-en+pl.model', torch_compile=True)

# pipe = Pipeline(s2a_ref='collabora/whisperspeech:s2a-q4-tiny-en+pl.model')

# works but slow, promising

# fileName = 'test2.wav'
# pipe.generate_to_file(fileName, """
# We are available to help you with both Open Source and proprietary AI projects
# """)

text = 'this is test file'

audio = pipe.generate(text)
output_audio = audio.cpu().numpy()

bytesData = output_audio.tobytes()
print(bytesData)