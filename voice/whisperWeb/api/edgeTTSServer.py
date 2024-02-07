from fastapi import FastAPI, WebSocket, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
from starlette.responses import FileResponse
import asyncio
import edge_tts
import time
import os
from pydantic import BaseModel

app = FastAPI()

class File(BaseModel):
    file: str

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_methods=["*"],
    allow_headers=["*"]
)

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
    return output_file

@app.get("/mp3/{file_path:path}")
async def read_mp3(file_path: str):
    return FileResponse(file_path, media_type="audio/mpeg")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        if data.startswith("play:"):
            text = data.split(":", 1)[1]
            file_path = await generate_voice(text)
            # Wait for the file to be fully generated before sending the URL
            while not os.path.exists(file_path):
                await asyncio.sleep(0.1)
            await websocket.send_text(f"http://localhost:8000/mp3/{file_path}")

@app.post("/delete")
async def delete_file(file: File):
    file_path = file.file.replace("http://localhost:8000/mp3/", "")
    print(file_path)
    if os.path.isfile(file_path):
        os.remove(file_path)

    return {"message": "File deleted"}


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
