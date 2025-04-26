#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSub(Node):

    def __init__(self):
        super().__init__("lidar_sub_node")

        # Subscriber to original /scan topic
        self.lidar_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_cb, 
            10
        )

        # Publisher to new /filtered_scan topic
        self.filtered_pub = self.create_publisher(
            LaserScan, 
            '/filtered_scan', 
            10
        )

    def lidar_cb(self, data):
        # Filter from 0 to 120 degrees (out of full 360°)
        total_samples = len(data.ranges)
        fov_samples = int(total_samples * (120.0 / 360.0))  # 120 degrees worth of samples
        
        filtered_scan = LaserScan()
        filtered_scan.header = data.header
        filtered_scan.angle_min = data.angle_min
        filtered_scan.angle_max = data.angle_min + (data.angle_increment * fov_samples)
        filtered_scan.angle_increment = data.angle_increment
        filtered_scan.time_increment = data.time_increment
        filtered_scan.scan_time = data.scan_time
        filtered_scan.range_min = data.range_min
        filtered_scan.range_max = data.range_max
        filtered_scan.ranges = data.ranges[0:fov_samples]
        filtered_scan.intensities = data.intensities[0:fov_samples]

        # Publish the filtered scan
        self.filtered_pub.publish(filtered_scan)

        # (Optional) Also print the filtered ranges
        self.get_logger().info(f"Filtered scan published with {fov_samples} samples.")

def main(args=None):
    rclpy.init(args=args)
    sensor_sub = LidarSub()
    rclpy.spin(sensor_sub)
    sensor_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
