from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import uvicorn
from starlette.responses import FileResponse
import edge_tts
import time
import os

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=["*"],
    allow_headers=["*"]
)

app.mount("/", StaticFiles(directory="index.html", html=True), name="ui")

async def generate_voice(text="Hello this is a test run", voice="en-US-SteffanNeural"):
    # Generate a timestamp for the output file name
    timestamp = time.strftime("%Y%m%d-%H%M%S")

    # Get the current working directory
    cwd = os.getcwd()
    output_file = os.path.join(cwd, f"{timestamp}.wav")
    print(output_file)
    communicate = edge_tts.Communicate(text, voice)
    with open(output_file, "wb") as file:
        async for chunk in communicate.stream():
            if chunk["type"] == "audio":
                file.write(chunk["data"])
            elif chunk["type"] == "WordBoundary":
                print(f"WordBoundary: {chunk}")


@app.get("/mp3")
async def read_mp3():
    file_path = 'D:\Robot/armController/20240206-191541.wav'
    return FileResponse(file_path, media_type="audio/mpeg")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        if data == "play":
            await websocket.send_text("http://localhost:8000/mp3")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
    
# @app.websocket("/ws")
# async def websocket_endpoint(websocket: WebSocket):
#     await websocket.accept()
#     while True:
#         data = await websocket.receive_text()
#         if data == "play":
#             file_path = 'D:\Robot/armController/20240205-000001.mp3'
#             # file_path = 'D:\Robot/armController/test2.wav'

#             with open(file_path, "rb") as f:
#                 while chunk := f.read(1024):
#                     await websocket.send_bytes(chunk)
#                     await asyncio.sleep(0.01)  # to prevent blocking other tasks

# @app.websocket("/audio")
# async def audio_stream(websocket: WebSocket):
#     await websocket.accept()
#     while True:
#         data = await get_next_audio_chunk()  # You need to implement this
#         await websocket.send_bytes(data)

# @app.websocket("/audio")
# async def audio_stream(websocket: WebSocket):
#     await websocket.accept()
#     async for audio_chunk in generate_voice():
#         await websocket.send_bytes(audio_chunk)

