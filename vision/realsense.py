import cv2
import numpy as np
import pyrealsense2 as rs

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

    # Show the images
    cv2.imshow('RGB Image', color_image)
    
    # show depth image
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)
    cv2.imshow('Depth Image', depth_colormap)
    
    # space key to exit
    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

pipeline.stop()
cv2.destroyAllWindows()
