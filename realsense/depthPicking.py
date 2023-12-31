import pyrealsense2 as rs
import cv2
import numpy as np

cv2.namedWindow("Colour Image", cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("Depth Image", cv2.WINDOW_AUTOSIZE)

pipeline = rs.pipeline()
config = rs.config()

wrapper = rs.pipeline_wrapper(pipeline)
profile = config.resolve(wrapper)

colorizer = rs.colorizer()
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

print(f"Depth scale:{depth_scale}")

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)

while True:
    frames = pipeline.wait_for_frames()
    frames = align.process(frames)
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()

    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.3), cv2.COLORMAP_JET)
    # Insert your object detection code here

    # I assume that I detected an object at [200, 200, 400, 400]

    # Draw the bounding box
    cv2.rectangle(color_image, (200, 200), (400, 400), (0, 255, 255), 1)
    cv2.rectangle(depth_image, (200, 200), (400, 400), (0, 255, 255), 1)

    depth = depth_image[200:400, 200:400].astype(float)

    depth = depth * depth_scale

    dist = cv2.mean(depth)

    cv2.putText(color_image, f"Depth: {dist} m", (200, 200), cv2.FONT_HERSHEY_PLAIN, 1.0, (0,0,0), 1)
    cv2.imshow("Colour Image", color_image)
    cv2.imshow("Depth Image", depth_cm)



    if cv2.waitKey(1) & 0xFF == 27: # Escape key closes the window(s)
        break

pipeline.stop()
cv2.destroyAllWindows()


# point cloud
# https://github.com/IntelRealSense/librealsense/tree/master/wrappers/opencv/kinfu

# pick and place
# https://roboticscasual.com/ros-tutorial-how-to-use-opencv-in-a-robot-pick-and-place-task-for-computer-vision/

# https://github.com/monkeyrom/3D_Object_Detection_and_Pose_Estimation_for_Automated_Bin-Picking_Application

# mrcnn
# https://pysource.com/2021/06/24/identify-and-measure-precisely-objects-distance-with-deep-learning-and-intel-realsense/

# tensorflow detection
# https://www.bing.com/videos/riverview/relatedvideo?q=intel%20realsense%20point%20cloud&mid=A70C3DE7523CA9FB598CA70C3DE7523CA9FB598C&ajaxhist=0