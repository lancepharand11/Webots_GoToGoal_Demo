# Go To Goal for Epuck robot in Webots
# Author: Lance Pharand 
# Provides necessary twist commands for robot to reach user desired goal 

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('my_robot_control')

        self.declare_parameter('des_x_loc', 0.5)
        self.declare_parameter('des_y_loc', 0.5)
        self.declare_parameter('Kp_lin_velo', 0.7)
        self.declare_parameter('Kp_ang_velo', 0.7)
        self.x1 = self.y1 = self.theta = 0

        self.velo_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.02, self.go_to_goal)

        self.imu_sub = self.create_subscription(Odometry, 'odom', self.pose_feedback, 10)

        self.get_logger().info('Robot Control Node has been started')
        self.get_logger().info('Reminder: Desired (x, y) location can be set using params des_x_loc and des_y_loc')

    def euclidean_dist(self, x1, x2, y1, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def angle_to_goal(self, x1, x2, y1, y2):
        return math.atan2(y2 - y1, x2 - x1)

    def adj_angle_limo(self, velo_command : Twist, Kp_ang_velo, angle_error):
        velo_command.linear.x = 0.0
        velo_command.angular.z = Kp_ang_velo * angle_error

    def move_limo(self, velo_command : Twist, Kp_lin_velo, euclidean_dist_val):
        velo_command.angular.z = 0.0
        velo_command.linear.x = Kp_lin_velo * euclidean_dist_val

    def stop_movement(self, velo_command : Twist):
        velo_command.linear.x = 0.0
        velo_command.angular.z = 0.0

    def go_to_goal(self):
        velo_pub_msg = Twist()

        x2 = float(self.get_parameter('des_x_loc').value)
        y2 = float(self.get_parameter('des_y_loc').value)
        Kp_lin_velo = float(self.get_parameter('Kp_lin_velo').value)
        Kp_ang_velo = float(self.get_parameter('Kp_ang_velo').value)

        angle_to_goal_val = self.angle_to_goal(self.x1, x2, self.y1, y2)

        # Errors for proportional control
        euclidean_dist_val = self.euclidean_dist(self.x1, x2, self.y1, y2)
        angle_error = angle_to_goal_val - self.theta

        # Tolerances
        dist_tol = 0.05  # m
        angle_tol = 0.1  # rad

        # Start by adjusting angle
        if (abs(angle_error) > angle_tol):
            self.adj_angle_limo(velo_pub_msg, Kp_ang_velo, angle_error)
        else:
            # Then move to the goal
            if (euclidean_dist_val > dist_tol):
                self.move_limo(velo_pub_msg, Kp_lin_velo, euclidean_dist_val)
            else:
                self.stop_movement(velo_pub_msg)
                self.get_logger().info(f"Desired loc ({x2}, {y2}) has been reached")

        self.velo_publisher.publish(velo_pub_msg)

    def pose_feedback(self, msg : Odometry):
        self.x1 = msg.pose.pose.position.x
        self.y1 = msg.pose.pose.position.y
        self.theta = 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
