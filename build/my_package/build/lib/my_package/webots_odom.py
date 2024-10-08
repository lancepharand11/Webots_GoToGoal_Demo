import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64  # For encoder readings
from geometry_msgs.msg import Twist
from tf_transformations import quaternion_from_euler

DISTANCE_BETWEEN_WHEELS = 0.090
WHEEL_RADIUS = 0.025

class WebotsOdomPublisher(Node):
    def __init__(self):
        super().__init__('webots_odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odometry) 

        # # Sub to the robots GPS (if used)
        # self.create_subscription(NavSatFix, 'gps', self.gps_callback, 10)
        
        # Sub to the robots IMU (for orientation)
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        
        # Subscribe to the encoders (for pose)
        self.create_subscription(Float64, 'left_wheel_encoder', self.left_encoder_callback, 10)
        self.create_subscription(Float64, 'right_wheel_encoder', self.right_encoder_callback, 10)
        
        # Encoder vars
        self.left_wheel_position = 0.0
        self.right_wheel_position = 0.0
        self.prev_left_wheel_position = 0.0
        self.prev_right_wheel_position = 0.0
        
        # Pose vars
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        
        self.last_time = self.get_clock().now()

    def left_encoder_callback(self, msg : Float64):
        self.left_wheel_position = msg.data

    def right_encoder_callback(self, msg : Float64):
        self.right_wheel_position = msg.data

    def gps_callback(self, msg : NavSatFix):
        # IF applicable, use GPS to update pose
        self.pose_x = msg.latitude  # Convert properly for your simulation
        self.pose_y = msg.longitude  # Convert properly for your simulation

    def imu_callback(self, msg : Imu):
        # Use IMU data to update orientation
        orientation_q = msg.orientation
        angular_velocity = msg.angular_velocity
        self.vel_angular = angular_velocity.z  # Update angular velocity from IMU
        _, _, self.pose_theta = quaternion_from_euler(0, 0, self.pose_theta)

    def compute_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Calculate change in encoder values
        delta_left = self.left_wheel_position - self.prev_left_wheel_position
        delta_right = self.right_wheel_position - self.prev_right_wheel_position
        
        self.prev_left_wheel_position = self.left_wheel_position
        self.prev_right_wheel_position = self.right_wheel_position

        # Calculate distance traveled by each wheel
        dist_left = delta_left * WHEEL_RADIUS
        dist_right = delta_right * WHEEL_RADIUS
        
        # Calculate linear and angular velocities
        linear_velocity = (dist_left + dist_right) / 2.0
        angular_velocity = (dist_right - dist_left) / DISTANCE_BETWEEN_WHEELS
        
        # Update the robot's pose (x, y, theta)
        delta_x = linear_velocity * dt * math.cos(self.pose_theta)
        delta_y = linear_velocity * dt * math.sin(self.pose_theta)
        delta_theta = angular_velocity * dt
        
        self.pose_x += delta_x
        self.pose_y += delta_y
        self.pose_theta += delta_theta

        return linear_velocity, angular_velocity

    def publish_odometry(self):
        linear_velocity, angular_velocity = self.compute_odometry()

        # Create the odom message
        current_time = self.get_clock().now()
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"

        # Set position
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0, 0, self.pose_theta)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Set velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Publish the message
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = WebotsOdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
