import socket
import sys
import cv2
import pickle
import numpy as np
import struct
import pyrealsense2 as rs

HOST=''
PORT=8089

s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print('Socket created')

s.bind((HOST,PORT))
print('Socket bind complete')
s.listen(10)
print('Socket now listening')

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)

while True:  # Add an outer loop to accept new connections
    conn,addr=s.accept()
    print('Connected with ' + addr[0] + ':' + str(addr[1]))

    data = b""
    payload_size = struct.calcsize("L") 
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        data = pickle.dumps(color_image)
        try:
            conn.sendall(struct.pack("L", len(data))+data)
        except ConnectionResetError:
            print('Client disconnected')
            break  # Break the inner loop if the client disconnects
