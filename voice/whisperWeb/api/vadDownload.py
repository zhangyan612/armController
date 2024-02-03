import os
import subprocess
import platform

def download(model_url="https://github.com/snakers4/silero-vad/raw/master/files/silero_vad.onnx"):
    target_dir = os.path.expanduser("~/.cache/whisper-live/")

    # Ensure the target directory exists
    os.makedirs(target_dir, exist_ok=True)

    # Define the target file path
    model_filename = os.path.join(target_dir, "silero_vad.onnx")

    # Check if the model file already exists
    if not os.path.exists(model_filename):
        # If it doesn't exist, download the model using appropriate command
        if platform.system() == "Windows":
            try:
                subprocess.run(["powershell", "-Command", f"wget {model_url} -OutFile {model_filename}"], check=True, shell=True)
            except subprocess.CalledProcessError:
                print("Failed to download the model using PowerShell.")
        else:
            try:
                subprocess.run(["wget", "-O", model_filename, model_url], check=True, shell=True)
            except subprocess.CalledProcessError:
                print("Failed to download the model using wget.")

    return model_filename

print(download())