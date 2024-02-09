import sounddevice as sd
import soundfile as sf 


sr = 44100
duration = 5
myrecording = sd.rec(int(duration * sr), samplerate=sr, channels=2)
sd.wait()  
sd.play(myrecording, sr)
sf.write("New Record.wav", myrecording, sr)
