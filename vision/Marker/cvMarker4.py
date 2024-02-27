import numpy as np
import cv2 as cv
from cv2 import aruco

file = "original.png"
np_img_3c = cv.imread(file)
np_img_result = np.copy(np_img_3c)

# "setup"
arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
arucoParams = aruco.DetectorParameters()

# "create the charuco board"
calibration_board = aruco.CharucoBoard((5, 7), squareLength=0.072, markerLength=0.062, dictionary=arucoDict)
board_image = calibration_board.generateImage((500, 700))
cv.imwrite("sample_board.png", board_image)

# "create charuco_detector"
charuco_detector = cv.aruco.CharucoDetector(calibration_board, detectorParams=arucoParams)

# "camera parameters"
fx = 1632.677490234375 
fy = 1632.677490234375 
cx = 640. 
cy = 480.
cameraMatrix = np.array([[fx, 0, cx], [0, fy, cy], [0.,0.,1.]])
distorsionCoeff = np.array([0.0, 0.0, 0.0, 0.0])

# "detect markers and interpolate corners"
charuco_corners, charuco_ids, marker_corners, marker_ids = charuco_detector.detectBoard(np_img_3c)

# "get pose of single markers"
estimateParameters = aruco.EstimateParameters()
estimateParameters.pattern = aruco.ARUCO_CW_TOP_LEFT_CORNER
#estimateParameters.pattern = aruco.ARUCO_CCW_CENTER  # set old axis

rvecs, tvecs, objPoints = aruco.estimatePoseSingleMarkers(marker_corners, markerLength=0.062, cameraMatrix=cameraMatrix,
                                                          distCoeffs=distorsionCoeff,
                                                          estimateParameters=estimateParameters)
img_single_pose = np.copy(np_img_3c)
for i in range(len(rvecs)):
    img_single_pose = cv.drawFrameAxes(img_single_pose, cameraMatrix, distorsionCoeff, rvecs[i], tvecs[i], 0.05)

cv.imwrite("4_calib_single_pose.png", img_single_pose)
