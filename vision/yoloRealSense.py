import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator

# Initialize YOLO and RealSense
model = YOLO('yolov8n.pt')

pipeline = rs.pipeline()
config = rs.config()
wrapper = rs.pipeline_wrapper(pipeline)
profile = config.resolve(wrapper)
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)

while True:
    # Get frames from RealSense
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()

    # Convert frames to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    # Use YOLO to detect objects
    results = model.predict(color_image)

    # Annotate each detected object
    for r in results:
        annotator = Annotator(color_image)
        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0]  # get box coordinates in (left, top, right, bottom) format
            c = box.cls

            # Calculate depth
            depth = depth_image[int(b[1]):int(b[3]), int(b[0]):int(b[2])].astype(float)
            depth = depth * depth_scale
            dist = cv2.mean(depth)[0]

            # Add depth info to the bounding box
            cv2.putText(color_image, f"Depth: {dist:.2f} m", (int(b[0]), int(b[1])-10), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,0,0), 1)
            annotator.box_label(b, model.names[int(c)] + f" Depth: {dist:.2f} m")

    # Show the images
    color_image = annotator.result()
    cv2.imshow('YOLO V8 Detection', color_image)
    
    # show depth image
    # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)
    # cv2.imshow('Depth Image', depth_colormap)
    
    # space key to exit
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

pipeline.stop()
cv2.destroyAllWindows()
