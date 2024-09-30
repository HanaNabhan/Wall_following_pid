# ~/ros2_workspace/src/wall_following_pkg/wall_following_pkg/lidar_processing_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class LidarProcessingNode(Node):
    def __init__(self):
        super().__init__('lidar_processing_node')
        self.lidar_ranges = None

        # Subscribe to LIDAR topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/autodrive/f1tenth_1/lidar',
            self.lidar_callback,
            20)
        
        # Publish distance to a custom topic
        self.distance_publisher = self.create_publisher(Float32, 'lidar_distance', 10)

        self.get_logger().info('Lidar Processing Node has been started.')

    def lidar_callback(self, msg):
        self.lidar_ranges = msg.ranges
        distance = self.get_right_wall_distance()
        
        # Publish the calculated distance
        distance_msg = Float32()
        distance_msg.data = distance
        self.distance_publisher.publish(distance_msg)

        self.get_logger().info(f'Received LIDAR data. Distance to right wall: {distance:.2f} meters')

    def get_right_wall_distance(self):
        if self.lidar_ranges is None:
            return float('inf')

        right_side_ranges = self.lidar_ranges[:len(self.lidar_ranges)//4]
        valid_ranges = [r for r in right_side_ranges if not np.isinf(r)]
        if valid_ranges:
            return min(valid_ranges)
        else:
            return float('inf')

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
