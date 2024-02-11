# from client import TranscriptionClient
# client = TranscriptionClient(
#   "localhost",
#   6006,
#   is_multilingual=False,
#   lang="en",
#   translate=False,
# )
# client()


from client import TranscriptionClient

if __name__ == "__main__":
    client = TranscriptionClient(
        "0.0.0.0", "6006", is_multilingual=False, lang="en", translate=False
    )
    client()   # uses microphone audio