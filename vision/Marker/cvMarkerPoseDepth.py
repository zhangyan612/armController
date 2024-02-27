import cv2
import pyrealsense2 as rs
import numpy as np

# print(cv2.__version__)

# marker with pose detection

matrix_coefficients = np.array(((933.15867, 0, 657.59), (0, 933.1586, 400.36993), (0, 0, 1)))
distortion_coefficients = np.array((-0.43948, 0.18514, 0, 0))

def main():
    # Initialize the RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    wrapper = rs.pipeline_wrapper(pipeline)
    profile = config.resolve(wrapper)
    depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

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
            depth_frame = frames.get_depth_frame()

            if not color_frame:
                continue

            if not depth_frame:
                continue

            # Convert the frame to a format compatible with OpenCV
            frame = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            # Detect ArUco markers
            detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            corners, ids, rejectedCandidates = detector.detectMarkers(frame)

            if np.all(ids is not None):  # If there are markers found by detector
                for i in range(0, len(ids)):  # Iterate in markers
                    objPoints = np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]])

                    valid, rvec, tvec = cv2.solvePnP(objPoints, corners[i], matrix_coefficients, distortion_coefficients)

                    # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
                    # pip install opencv-contrib-python
                    rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.02, matrix_coefficients, distortion_coefficients)
                    (rvec - tvec).any()  # get rid of that nasty numpy value array error
                    cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
                    cv2.drawFrameAxes(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis

                    # Calculate the center of the marker
                    cX = int((corners[i][0][0][0] + corners[i][0][2][0]) / 2)
                    cY = int((corners[i][0][0][1] + corners[i][0][2][1]) / 2)

                    # Get the depth value using the center point
                    depth_value = depth_image[cY, cX] * depth_scale

                    # Print the depth value in meters
                    # print(f"Marker ID {ids[i][0]} Depth: {depth_value:.3f} meters")

                    # add to the marker
                    # Define the font for the text to be displayed
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    # Prepare the text with the depth information
                    depth_info = f"ID {ids[i][0]} Depth: {depth_value:.3f}m"
                    # Define the position for the text (slightly above the center of the marker)
                    text_position = (cX, cY - 15)
                    # Draw the text on the frame
                    cv2.putText(frame, depth_info, text_position, font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)


            # Display the resulting frame
            cv2.imshow('frame', frame)

            # Press 'q' to exit
            if cv2.waitKey(1) & 0xFF == ord(' '):
                break

    finally:
        # Release the RealSense pipeline and close the window
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
