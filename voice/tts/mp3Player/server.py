from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import base64
import uvicorn
from starlette.responses import FileResponse
import asyncio
import edge_tts

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=["*"],
    allow_headers=["*"]
)

# @app.get("/mp3")
# async def read_mp3():
#     file_path = 'D:\Robot/armController/20240204-235950.wav'
#     return FileResponse(file_path, media_type="audio/mpeg")

# @app.websocket("/ws")
# async def websocket_endpoint(websocket: WebSocket):
#     await websocket.accept()
#     while True:
#         data = await websocket.receive_text()
#         if data == "play":
#             await websocket.send_text("http://localhost:8000/mp3")


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

async def generate_voice(text="Hello this is a test run", voice="en-US-SteffanNeural"):
    communicate = edge_tts.Communicate(text, voice)
    audio_data = b""
    async for chunk in communicate.stream():
        if chunk["type"] == "audio":
            yield chunk["data"]
        elif chunk["type"] == "WordBoundary":
            print(f"WordBoundary: {chunk}")

@app.websocket("/audio")
async def audio_stream(websocket: WebSocket):
    await websocket.accept()
    async for audio_chunk in generate_voice():
        await websocket.send_bytes(audio_chunk)



if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)