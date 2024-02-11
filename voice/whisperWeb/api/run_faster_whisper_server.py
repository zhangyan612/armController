from transcription_server_update import TranscriptionServer

from multiprocessing import Event

tts_playing_event = Event()
server = TranscriptionServer()
server.run("0.0.0.0", 6006, None, None, tts_playing_event)