import cv2
import pyrealsense2 as rs
import numpy as np

def main():
    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    # Start the pipeline
    pipeline.start(config)

    # Load the ArUco dictionary (you can choose a different one if needed)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    # Define ArUco detection parameters
    aruco_params = cv2.aruco.DetectorParameters()

    try:
        while True:
            # Wait for a new frame from the RealSense camera
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert the frame to a format compatible with OpenCV
            frame = np.asanyarray(color_frame.get_data())

            # Detect ArUco markers
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, rejectedCandidates = detector.detectMarkers(frame)

            # Draw detected markers on the frame
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Display the frame
            cv2.imshow("ArUco Marker Detection", frame)

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord(' '):
                break

    finally:
        # Release the RealSense pipeline and close the window
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
