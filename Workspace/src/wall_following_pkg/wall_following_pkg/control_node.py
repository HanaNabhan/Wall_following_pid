# ~/ros2_workspace/src/wall_following_pkg/wall_following_pkg/control_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.desired_distance_right = 0.64  # Desired distance from the right wall in meters
        # Change this to match your vehicle
        self.kp_steering = 0.0
        self.ki_steering = 0.0
        self.kd_steering = 0.0
        self.kp_throttle = 0.0
        self.max_throttle = 0.00

        self.last_error = 0.0
        self.integral = 0.0

        qos_profile = QoSProfile(depth=10)

        # Publishers for steering and throttle commands
        self.steering_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/steering_command', qos_profile)
        self.throttle_publisher = self.create_publisher(Float32, '/autodrive/f1tenth_1/throttle_command', qos_profile)

        # Timer to call control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Subscribe to the Lidar Processing Node's distance data
        self.lidar_subscription = self.create_subscription(
            Float32,
            'lidar_distance',
            self.lidar_callback,
            qos_profile)

        self.current_distance = float('inf')

        self.get_logger().info('Control Node has been started.')

    def lidar_callback(self, msg):
        self.current_distance = msg.data

    def control_loop(self):
        if self.current_distance == float('inf'):
            return

        error = self.desired_distance_right - self.current_distance
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error

        # Steering control
        steering_control = self.kp_steering * error + self.ki_steering * self.integral + self.kd_steering * derivative
        steering_control = max(-1.0, min(1.0, steering_control))
        steering_msg = Float32()
        steering_msg.data = steering_control
        self.steering_publisher.publish(steering_msg)

        # Throttle control
        throttle_control = self.max_throttle - (self.kp_throttle * abs(error))
        throttle_control = max(0.01, min(self.max_throttle, throttle_control))
        throttle_control = 0.07  # Fixed throttle value
        throttle_msg = Float32()
        throttle_msg.data = throttle_control
        self.throttle_publisher.publish(throttle_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
