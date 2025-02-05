import cv2 as cv
import os
import numpy as np

# Checkerboard size
CHESS_BOARD_DIM = (8, 6)

# The size of a square in the checkerboard (in mm)
SQUARE_SIZE = 24  

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Directory to store calibration data
calib_data_path = "/home/jalil/catkin_ws3/src/aruco_image/calib_data"
if not os.path.isdir(calib_data_path):
    os.makedirs(calib_data_path)
    print(f'"{calib_data_path}" Directory is created')
else:
    print(f'"{calib_data_path}" Directory already Exists.')

# Prepare object points
obj_3D = np.zeros((CHESS_BOARD_DIM[0] * CHESS_BOARD_DIM[1], 3), np.float32)
obj_3D[:, :2] = np.mgrid[0:CHESS_BOARD_DIM[0], 0:CHESS_BOARD_DIM[1]].T.reshape(-1, 2)
obj_3D *= SQUARE_SIZE
print(obj_3D)

# Arrays to store object points and image points
obj_points_3D = []  
img_points_2D = []  

# The images directory path
image_dir_path = "/home/jalil/catkin_ws3/src/aruco_image/images"
files = os.listdir(image_dir_path)

grayScale = None  # Initialize to avoid NameError

for file in files:
    print(f"Processing: {file}")
    imagePath = os.path.join(image_dir_path, file)
    image = cv.imread(imagePath)

    if image is None:
        print(f"Warning: Could not read {imagePath}. Skipping...")
        continue

    grayScale = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(grayScale, CHESS_BOARD_DIM, None)

    if ret:
        obj_points_3D.append(obj_3D)
        corners2 = cv.cornerSubPix(grayScale, corners, (3, 3), (-1, -1), criteria)
        img_points_2D.append(corners2)
        image = cv.drawChessboardCorners(image, CHESS_BOARD_DIM, corners2, ret)

# Ensure we have valid calibration data
if len(obj_points_3D) == 0 or len(img_points_2D) == 0:
    print("Error: No valid chessboard images found. Calibration aborted.")
    exit()

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(
    obj_points_3D, img_points_2D, grayScale.shape[::-1], None, None
)

print("Calibration completed.")

# Save calibration data
np.savez(f"{calib_data_path}/MultiMatrix", camMatrix=mtx, distCoef=dist, rVector=rvecs, tVector=tvecs)
print("Calibration data saved successfully.")

# Load and verify saved data
data = np.load(f"{calib_data_path}/MultiMatrix.npz")
camMatrix = data["camMatrix"]
distCoef = data["distCoef"]
rVector = data["rVector"]
tVector = data["tVector"]

print("Loaded calibration data successfully.")
