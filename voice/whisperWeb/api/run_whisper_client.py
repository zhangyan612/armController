from whisper_live.client import TranscriptionClient
client = TranscriptionClient(
  "localhost",
  6006,
  is_multilingual=False,
  lang="en",
  translate=False,
)
client()
