#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class ArucoPIDController:
    def __init__(self):
        rospy.init_node('aruco_pid_controller', anonymous=True)
        self.sub_distance = rospy.Subscriber('dist_pub/', Float32, self.distance_callback)
        self.sub_angle = rospy.Subscriber('/angle_pub', Float32, self.angle_callback)
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.pid_distance = PIDController(kp=0.5, ki=0.0, kd=0.1)
        self.pid_angle = PIDController(kp=0.4, ki=0.0, kd=0.1)
        self.target_distance = 0.05  # 5 cm
        self.last_time = rospy.Time.now().to_sec()

        self.current_distance_error = 0.0
        self.current_angle_error = 0.0

    def distance_callback(self, msg):
        self.current_distance_error = msg.data - self.target_distance
        self.control_loop()

    def angle_callback(self, msg):
        self.current_angle_error = msg.data
        self.control_loop()

    def control_loop(self):
        current_time = rospy.Time.now().to_sec()
        dt = current_time - self.last_time
        self.last_time = current_time

        linear_vel = self.pid_distance.compute(self.current_distance_error, dt)
        angular_vel = self.pid_angle.compute(self.current_angle_error, dt)

        cmd = Twist()
        cmd.linear.x = max(min(linear_vel, 0.2), -0.2)  # Limit speed
        cmd.angular.z = max(min(angular_vel, 0.5), -0.5)  # Limit rotation

        self.pub_cmd_vel.publish(cmd)

if __name__ == '__main__':
    try:
        controller = ArucoPIDController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

