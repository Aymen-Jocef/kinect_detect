#!/usr/bin/env python3
import rospy
import cv2 as cv
from cv2 import aruco
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Load calibration data
calib_data_path = "/home/jalil/catkin_ws3/src/aruco_image/scripts/calib_data/MultiMatrix.npz"
calib_data = np.load(calib_data_path)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]

# Marker settings
MARKER_SIZE = 5  # centimeters
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
param_markers = aruco.DetectorParameters()
detector = aruco.ArucoDetector(marker_dict, param_markers)

# Initialize CvBridge
bridge = CvBridge()

def image_callback(msg):
    """ Callback function to process the image from the Kinect """
    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr(f"cv_bridge Error: {e}")
        return

    height, width, _ = frame.shape
    z_13 = []
    x_13 = []
    z_36 = []
    x_36 = []
    min_z_13 = None
    min_z_36 = None
    min_distance = None
    min_depth = None

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, ids, reject = detector.detectMarkers(gray_frame)

    if ids is not None and len(ids) > 0:
        for i in range(len(ids)):
            if ids[i] == 0 or ids[i] == 36:
                obj_points = np.array([[0, 0, 0],
                                       [MARKER_SIZE, 0, 0],
                                       [MARKER_SIZE, MARKER_SIZE, 0],
                                       [0, MARKER_SIZE, 0]], dtype=np.float32)
                _, rVec, tVec = cv.solvePnP(obj_points, marker_corners[i], cam_mat, dist_coef)

                corners = marker_corners[i].reshape(4, 2)
                corners = corners.astype(int)
                marker_center = tuple(np.mean(corners, axis=0).astype(int))

                distance_x1 = width // 2 - marker_center[0]
                z_value = tVec[2][0]

                if ids[i] == 0:
                    z_13.append(z_value)
                    min_z_13 = min(z_13)
                    index_min_z_13 = z_13.index(min_z_13)
                    x_13.append(distance_x1)
                    min_distance_13 = x_13[index_min_z_13]
                elif ids[i] == 36:
                    z_36.append(z_value)
                    min_z_36 = min(z_36)
                    index_min_z_36 = z_36.index(min_z_36)
                    x_36.append(distance_x1)
                    min_distance_36 = x_36[index_min_z_36]

        if min_z_36 is not None and min_z_13 is not None:
            if min_z_13 < min_z_36:
                min_depth = min_z_13
                min_distance = min_distance_13
            elif min_z_36 < min_z_13:
                min_depth = min_z_36
                min_distance = min_distance_36
        elif min_z_13 is not None:
            min_depth = min_z_13
            min_distance = min_distance_13
        elif min_z_36 is not None:
            min_depth = min_z_36
            min_distance = min_distance_36

        rospy.loginfo(f"Closest Marker Depth: {min_depth}")

    cv.imshow("ArUco Detection", frame)
    cv.waitKey(1)

def main():
    rospy.init_node("aruco_detector", anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.loginfo("Aruco Detector Node Started")
    rospy.spin()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
