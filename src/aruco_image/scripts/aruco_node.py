#!/usr/bin/env python3
import rospy
import cv2 as cv
from cv2 import aruco
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32  # Added for distance publishing
from cv_bridge import CvBridge

# Load calibration data
calib_data_path = "/root/kinect_for_eurobot/src/aruco_image/scripts/calib_data/MultiMatrix.npz"
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

# ROS Publisher
dist_pub = None  # Will be initialized in main()

def image_callback(msg):
    """ Callback function to process the image from the Kinect """
    global dist_pub

    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr(f"cv_bridge Error: {e}")
        return

    height, width, _ = frame.shape
    min_z_13 = None
    min_z_36 = None

    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, ids, _ = detector.detectMarkers(gray_frame)

    if ids is not None and len(ids) > 0:
        for i in range(len(ids)):
            if ids[i] == 0 or ids[i] == 36:
                obj_points = np.array([[0, 0, 0],
                                       [MARKER_SIZE, 0, 0],
                                       [MARKER_SIZE, MARKER_SIZE, 0],
                                       [0, MARKER_SIZE, 0]], dtype=np.float32)
                _, rVec, tVec = cv.solvePnP(obj_points, marker_corners[i], cam_mat, dist_coef)

                z_value = tVec[2][0]  # Extract the Z distance (depth)

                if ids[i] == 0:
                    if min_z_13 is None or z_value < min_z_13:
                        min_z_13 = z_value
                elif ids[i] == 36:
                    if min_z_36 is None or z_value < min_z_36:
                        min_z_36 = z_value

    # Determine the closest marker distance
    min_depth = None
    if min_z_13 is not None and min_z_36 is not None:
        min_depth = min(min_z_13, min_z_36)
    elif min_z_13 is not None:
        min_depth = min_z_13
    elif min_z_36 is not None:
        min_depth = min_z_36

    # Publish the distance if a marker was detected
    if min_depth is not None:
        rospy.loginfo(f"Closest Marker Depth: {min_depth:.2f} cm")
        dist_pub.publish(min_depth)  # Publish the distance

    # Display the camera feed with markers
    cv.imshow("ArUco Detection", frame)
    cv.waitKey(1)

def main():
    global dist_pub

    rospy.init_node("aruco_detector", anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    # Initialize the publisher
    dist_pub = rospy.Publisher("/dist_pub", Float32, queue_size=10)

    rospy.loginfo("Aruco Detector Node Started")
    rospy.spin()

if __name__ == "__main__":
    main()

