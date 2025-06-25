from pynput import keyboard

def on_press(key):
    if key == keyboard.Key.end:
        print("End key pressed. Exiting...")
        listener.stop()  # This will exit the program

listener = keyboard.Listener(on_press=on_press)
listener.start()
print("Program running. Press End key to exit...")
listener.join()  # Wait until listener stops