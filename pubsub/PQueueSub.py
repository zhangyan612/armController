import persistqueue
import time

q = persistqueue.SQLiteQueue('audio', auto_commit=True)

while True:
    print(q.get())
    time.sleep(0.1)