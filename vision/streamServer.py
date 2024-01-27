# Server side
import cv2
import socket
import pickle
import struct

cap = cv2.VideoCapture(0)
serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serversocket.bind(('localhost', 8089))
serversocket.listen(10)

while True:
    conn, addr = serversocket.accept()
    ret, frame = cap.read()
    data = pickle.dumps(frame)
    conn.sendall(struct.pack("L", len(data)) + data)
