#!/usr/bin/env python
import persistqueue
import time

def listener(topic):
    q = persistqueue.SQLiteQueue(topic, auto_commit=True)
    while True:
        print(q.get())
        time.sleep(0.1)

if __name__ == '__main__':
    listener('vision')
