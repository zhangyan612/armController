import subprocess

duration = 5  # seconds
wav_file = "test_record.wav"

print("Recording...")
subprocess.run(["arecord", "-d", str(duration), "-f", "cd", wav_file])
print("Recording saved to", wav_file)

print("Playing back...")
subprocess.run(["aplay", wav_file])
print("Done.")
