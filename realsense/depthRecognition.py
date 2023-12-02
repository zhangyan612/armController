import cv2
import pyrealsense2 as rs
from mask_rcnn import *
# Load Realsense camera and Mask R-CNN

mrcnn = MaskRCNN()

while True:
    # Get frame in real time from Realsense camera
    ret, bgr_frame, depth_frame = rs.get_frame_stream()
    cv2.imshow("Bgr frame", bgr_frame)

    # Get object mask
    boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame)
    # Draw object mask
    bgr_frame = mrcnn.draw_object_mask(bgr_frame)
