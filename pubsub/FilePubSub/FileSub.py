# reader.py
import time
import os
import json

def subscribe_to_change(key):
    filename = 'state.json'
    last_modified = os.path.getmtime(filename)
    last_value = None
    changed = False
    while True:
        if os.path.getmtime(filename) != last_modified:
            last_modified = os.path.getmtime(filename)
            with open(filename, 'r') as f:
                new_contents = json.load(f)
            new_value = new_contents.get(key, None)
            if new_value is not None and new_value != last_value:
                print(f"{key}: {new_value}")
                changed = True
                last_value = new_value
        time.sleep(1)

subscribe_to_change('data')
