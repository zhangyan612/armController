import time
import requests
import pyautogui

CF_WORKER_URL = 'https://mousestate.zhangyan612.workers.dev'

last_pos = {'x': None, 'y': None}

while True:
    try:
        resp = requests.get(f"{CF_WORKER_URL}/latest", timeout=10)
        if resp.status_code == 200:
            data = resp.json()
            x, y = data.get('x'), data.get('y')
            if x is not None and y is not None:
                # Only move if position changed
                if (x, y) != (last_pos['x'], last_pos['y']):
                    pyautogui.moveTo(x, y)
                    last_pos['x'], last_pos['y'] = x, y
    except Exception as e:
        print(f"Receive error: {e}")
    time.sleep(0.02)  # 50 Hz polling