# client
import cv2
import numpy as np
import socket
import sys
import pickle
import struct
cap=cv2.VideoCapture(0)
clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('localhost',8088))
while True:
    ret,frame=cap.read()
    # Display the resulting frame
    cv2.imshow('Video', frame)
    if cv2.waitKey(1) & 0xFF == ord(' '):  # Press Q on the keyboard to exit
        break
    data = pickle.dumps(frame)
    clientsocket.sendall(struct.pack("L", len(data))+data)
cap.release()
cv2.destroyAllWindows()  # After the loop release the cap object and destroy all windows
