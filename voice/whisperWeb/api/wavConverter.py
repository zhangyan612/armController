from pydub import AudioSegment

def convert_wav49_to_wav(input_file, output_file):
    audio = AudioSegment.from_file(input_file, format="wav")
    audio.export(output_file, format="wav")

# Usage
convert_wav49_to_wav("D:\Robot/armController/20240205-000001.wav", "D:\Robot/armController/20240204-235950.wav")
