from openai import OpenAI
import configparser

config = configparser.ConfigParser()
config.read('.env')
LEPTON_WORKSPACE_TOKEN = config.get('LEPTON', 'token')

client = OpenAI(
    base_url=f"https://latest-whisperx.cloud.lepton.ai",
    api_key=LEPTON_WORKSPACE_TOKEN,
)

audio_file= open("D:\Robot/armController/20240204-234829.wav", "rb")
transcript = client.audio.transcriptions.create(
  model="whisperX", 
  file=audio_file
)

print(transcript)


# from leptonai.client import Client
# from leptonai.photon import File
# # import base64

# c = Client("https://latest-whisperx.cloud.lepton.ai", token=LEPTON_WORKSPACE_TOKEN)


# with open("D:\Robot/armController/20240204-234829.wav", "rb") as f:
#     result = c.run(input=File(f))
#     print(result[0]["text"])

