import cv2
import numpy as np
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

align = rs.align(rs.stream.color)

checkerboard_size = (3,3)

while True:
    # Capture a frame from the RealSense camera
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    color_image = np.asanyarray(color_frame.get_data())

    # Find chessboard corners in the current frame
    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    if ret:
        # Refine the corners
        corners2 = cv2.cornerSubPix(gray, corners, checkerboard_size, (-1, -1), criteria)

        # Draw the corners on the color image
        cv2.drawChessboardCorners(color_image, checkerboard_size, corners2, ret)

    # Display the modified frame
    cv2.imshow('RGB Image with Chessboard Corners', color_image)
        
    # space key to exit
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

pipeline.stop()
cv2.destroyAllWindows()
