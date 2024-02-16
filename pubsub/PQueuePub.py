import persistqueue
q = persistqueue.SQLiteQueue('audio', auto_commit=True)

q.put('test new audio')
q.put('this should generate a second file')
q.put('third file')

# print(q.get())