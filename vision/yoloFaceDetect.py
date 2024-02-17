import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
import time
import face_recognition

# Initialize YOLO and RealSense
model = YOLO("yolov8n-face.pt")

pipeline = rs.pipeline()
config = rs.config()
wrapper = rs.pipeline_wrapper(pipeline)
profile = config.resolve(wrapper)

config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)

last_saved_time = time.time()

# Load known faces
known_faces = []
known_names = []
# Add your known faces and names here
# For example:
known_image = face_recognition.load_image_file("face_1708150594.png")
known_faces.append(face_recognition.face_encodings(known_image)[0])
known_names.append('Yan')

while True:
    # Get frames from RealSense
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()

    # Convert frames to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())

    # Use YOLO to detect objects
    results = model.predict(color_image)

    # Annotate each detected object
    for r in results:
        annotator = Annotator(color_image)
        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
            c = box.cls

            # Save the face to an image file every 10 seconds
            if time.time() - last_saved_time >= 10:
                face = color_image[int(b[1]):int(b[3]), int(b[0]):int(b[2])]
                cv2.imwrite(f'face_{int(time.time())}.png', face)
                last_saved_time = time.time()

            # Compare the detected face with known faces
            unknown_encoding = face_recognition.face_encodings(face)
            results = face_recognition.compare_faces(known_faces, unknown_encoding[0])
            name = "Unknown"
            if True in results:
                name = known_names[results.index(True)]

            # Add depth info to the bounding box
            cv2.putText(color_image, name, (int(b[0]), int(b[1])-10), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,0,0), 1)
            annotator.box_label(b, model.names[int(c)] + f" {name}")

    # Show the images
    color_image = annotator.result()
    cv2.imshow('YOLO V8 Detection', color_image)
        
    # space key to exit
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

pipeline.stop()
cv2.destroyAllWindows()
