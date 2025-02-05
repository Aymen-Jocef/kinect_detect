import cv2 as cv
import os
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

CHESS_BOARD_DIM = (8, 6)
image_dir_path = "/home/jalil/catkin_ws3/src/aruco_image/images"
n = 0  # image counter

# Create images directory if not exists
if not os.path.isdir(image_dir_path):
    os.makedirs(image_dir_path)
    print(f'"{image_dir_path}" Directory is created')
else:
    print(f'"{image_dir_path}" Directory already exists.')

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
bridge = CvBridge()
board_detected = False
gray = None
frame = None

def detect_checker_board(image, grayImage, criteria, boardDimension):
    ret, corners = cv.findChessboardCorners(grayImage, boardDimension)
    if ret:
        corners1 = cv.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
        image = cv.drawChessboardCorners(image, boardDimension, corners1, ret)
    return image, ret

def image_callback(msg):
    global frame, gray, board_detected
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    frame, board_detected = detect_checker_board(frame, gray, criteria, CHESS_BOARD_DIM)

def save_image():
    global n
    if board_detected and frame is not None:
        filename = f"{image_dir_path}/image{n}.png"
        cv.imwrite(filename, frame)
        print(f"Saved image number {n}: {filename}")
        n += 1

def main():
    rospy.init_node('image_capture', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    
    print("Press 's' to save an image, 'q' to quit.")
    while not rospy.is_shutdown():
        if frame is not None:
            display_frame = frame.copy()
            cv.putText(display_frame, f"saved_img: {n}", (30, 40), cv.FONT_HERSHEY_PLAIN, 1.4, (0, 255, 0), 2, cv.LINE_AA)
            cv.imshow("frame", display_frame)
        
        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            save_image()
    
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
