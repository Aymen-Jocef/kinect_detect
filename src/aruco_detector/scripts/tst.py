import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def pointcloud_callback(msg):
    width = msg.width
    height = msg.height
    middle_index = (width // 2, height // 2)

    point_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    points = list(point_gen)

    if len(points) > 0:
        middle_point = points[width * (height // 2) + (width // 2)]
        depth_value = middle_point[2]  # Z value is depth
        print(f"Depth at center: {depth_value} meters")

rospy.init_node("depth_reader", anonymous=True)
rospy.Subscriber("/camera/depth/points", PointCloud2, pointcloud_callback)
rospy.spin()


