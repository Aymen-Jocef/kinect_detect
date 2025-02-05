import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # Subscribe to the RGB and Depth topics
        self.depth_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.depth_callback)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        
        self.image_pub = rospy.Publisher("/aruco_detected_image", Image, queue_size=1)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.parameters = cv2.aruco.DetectorParameters()
        
        self.latest_depth_image = None  # Store the latest depth image
        
        rospy.loginfo("Aruco Detector Node Started")

    def depth_callback(self, msg):
        """Store the latest depth image"""
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

    def aruco_display(self, corners, ids, rejected, image):
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                # Draw marker outline
                cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

                # Compute center of marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)

                # Draw a circle at the center
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

                # Get the depth value at (cX, cY)
                if self.latest_depth_image is not None:
                    depth_value = self.latest_depth_image[cY, cX]
                    print(depth_value)
                    rospy.loginfo(f"ArUco ID {markerID} - Depth: {depth_value} mm")

                cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                cv2.putText(image, str(cY), (topLeft[0], topLeft[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        return image

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Detect Aruco markers
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

            # Draw and display ArUco markers
            image = self.aruco_display(corners, ids, rejected, cv_image)

            # Publish processed image
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

            # Show the image
            cv2.imshow("Aruco Marker Detection", image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()
if __name__ == '__main__':
    detector = ArucoDetector()
    detector.run()