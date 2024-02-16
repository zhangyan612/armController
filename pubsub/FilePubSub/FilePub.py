# writer.py
import time
import json

def updateState(key, value):
    with open('state.json', 'w') as f:
        json.dump({'time': time.ctime(), key: value}, f)


updateState('audio', 'keep testing audio')