#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import rclpy.time
from sensor_msgs.msg import CameraInfo, Image

class CameraNode(Node):

        def __init__(self):
              super().__init__("camera")
              self.cam_img_pub_ = self.create_publisher(Image, "/camera/image_raw", 10)
              self.cam_info_pub_ = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

              self.get_logger().info("The camera has started")

              self.brigde = CvBridge()

              self.timer = self.create_timer(0.1, self.timer_callback)

              self.cap = cv2.VideoCapture(0)

              self.cam_info = CameraInfo()
              self.cam_info.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
              self.cam_info.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
              self.cam_info.distortion_model = "plumb_bob"

              self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
              self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
              self.fps = self.cap.get(cv2.CAP_PROP_FPS)


        # aruco identification, display the ids 
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
            
                        cv2.line(image, topLeft, topRight, (0, 0, 255), 2)
                        cv2.line(image, topRight, bottomRight, (0, 0, 255), 2)
                        cv2.line(image, bottomRight, bottomLeft, (0, 0, 255), 2)
                        cv2.line(image, bottomLeft, topLeft, (0, 0, 255), 2)

                        # draw a dot in the midle of the markers
                        #cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                        #cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                        #cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
            
                        cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 255, 0), 2)
                        print("[Inference] ArUco marker ID: {}".format(markerID))
            
                return image

        def timer_callback(self):
              ret, frame = self.cap.read()
              if ret :
                    
                    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
                    parameters =  cv2.aruco.DetectorParameters()
                    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

                    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)

                    detected_markers = self.aruco_display(markerCorners, markerIds, rejectedCandidates, frame)
 
                    cv2.imshow("Markers", detected_markers)

                     # converting image to ROS msg
                    self.ros_msg = self.brigde.cv2_to_imgmsg(frame, encoding="bgr8") 
                    # publishing the frame (the image from the camera)                  
                    self.cam_img_pub_.publish(self.ros_msg)


                    self.cam_info.header = self.ros_msg.header
                  
                    self.cam_info_pub_.publish(self.cam_info)
                    # cv2.imshow("frame", frame)
                    cv2.waitKey(1)
              else:
                    self.get_logger().warn("Failed to read frame")
              
              
        def destroy_node(self):
              self.cap.release()
              cv2.destroyAllWindows()
              super().destroy_node()

        
        
            

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()