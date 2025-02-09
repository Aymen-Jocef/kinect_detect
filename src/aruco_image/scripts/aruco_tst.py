#!/usr/bin/env python3
import rospy
import cv2 as cv
from cv2 import aruco
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32  # For distance & angle publishing
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

# ROS Publishers
dist_pub = None  
angle_pub = None  

def rotation_vector_to_euler(rVec):
    """ Converts a rotation vector to Euler angles (yaw, pitch, roll). """
    R, _ = cv.Rodrigues(rVec)  # Convert rVec to a 3x3 rotation matrix

    # Extract Euler angles from rotation matrix
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    
    if sy < 1e-6:  # Prevent division by zero
        yaw = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        roll = 0
    else:
        yaw = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        roll = np.arctan2(R[1, 0], R[0, 0])

    # Convert radians to degrees
    yaw_deg = np.degrees(yaw)
    pitch_deg = np.degrees(pitch)
    roll_deg = np.degrees(roll)
    
    return yaw_deg, pitch_deg, roll_deg  # Return angles in degrees

def image_callback(msg):
    """ Callback function to process the image from the Kinect """
    global dist_pub, angle_pub

    try:
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr(f"cv_bridge Error: {e}")
        return

    height, width, _ = frame.shape
    min_z_13 = None
    min_z_36 = None
    best_yaw = None  

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
                yaw, _, _ = rotation_vector_to_euler(rVec)  # Extract yaw angle (horizontal rotation)

                if ids[i] == 0:
                    if min_z_13 is None or z_value < min_z_13:
                        min_z_13 = z_value
                        best_yaw = yaw
                elif ids[i] == 36:
                    if min_z_36 is None or z_value < min_z_36:
                        min_z_36 = z_value
                        best_yaw = yaw

    # Determine the closest marker distance
    min_depth = None
    if min_z_13 is not None and min_z_36 is not None:
        min_depth = min(min_z_13, min_z_36)
    elif min_z_13 is not None:
        min_depth = min_z_13
    elif min_z_36 is not None:
        min_depth = min_z_36

    # Publish distance if available
    if min_depth is not None:
        rospy.loginfo(f"Closest Marker Depth: {min_depth:.2f} cm")
        dist_pub.publish(min_depth)

    # Publish yaw angle if available
    if best_yaw is not None:
        rospy.loginfo(f"Camera Yaw Angle: {best_yaw:.2f} degrees")
        angle_pub.publish(best_yaw)

    # Display the camera feed with markers
    cv.imshow("ArUco Detection", frame)
    cv.waitKey(1)

def main():
    global dist_pub, angle_pub

    rospy.init_node("aruco_detector", anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)

    # Initialize the publishers
    dist_pub = rospy.Publisher("/dist_pub", Float32, queue_size=10)
    angle_pub = rospy.Publisher("/angle_pub", Float32, queue_size=10)

    rospy.loginfo("Aruco Detector Node Started")
    rospy.spin()

if __name__ == "__main__":
    main()

