import time
import requests
from pynput import mouse

CF_WORKER_URL = 'https://mousestate.zhangyan612.workers.dev'

current_pos = {"x": 0, "y": 0}

def on_move(x, y):
    global current_pos
    current_pos['x'], current_pos['y'] = int(x), int(y)

# Start listener
listener = mouse.Listener(on_move=on_move)
listener.start()

# Loop and send updates at fixed interval
def main():
    while True:
        try:
            print(f"Sending: {current_pos}")
            resp = requests.post(f"{CF_WORKER_URL}/update", json=current_pos, timeout=10)
            if resp.status_code != 200:
                print(f"Error: {resp.status_code}")
        except Exception as e:
            print(f"Send error: {e}")
        time.sleep(0.02)  # 50 Hz update rate

if __name__ == '__main__':
    main()