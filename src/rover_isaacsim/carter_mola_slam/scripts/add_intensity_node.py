#!/usr/bin/env python3
"""
ROS2 Node: Add Intensity to Carter LiDAR PointCloud2
Subscribes to /carter/lidar (x,y,z only)
Publishes to /carter/lidar_with_intensity (x,y,z,intensity)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np
import struct


class AddIntensityNode(Node):
    def __init__(self):
        super().__init__('add_intensity_node')

        # Subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            '/carter/lidar',
            self.lidar_callback,
            10
        )

        # Publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            '/carter/lidar_with_intensity',
            10
        )

        self.get_logger().info('Add Intensity Node started')
        self.get_logger().info('Subscribing to: /carter/lidar')
        self.get_logger().info('Publishing to: /carter/lidar_with_intensity')

    def lidar_callback(self, msg):
        """Process incoming PointCloud2 and add intensity field"""

        # Read points (x, y, z)
        points_list = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        if len(points_list) == 0:
            self.get_logger().warn('Received empty point cloud')
            return

        points = np.array(points_list)

        # Calculate intensity based on distance
        # Formula: intensity = 100.0 / (1.0 + distance)
        # Closer points have higher intensity
        distances = np.linalg.norm(points, axis=1)
        intensities = 100.0 / (1.0 + distances)

        # Create new PointCloud2 with intensity
        new_points = []
        for i in range(len(points)):
            x, y, z = points[i]
            intensity = intensities[i]
            new_points.append([x, y, z, intensity])

        # Define fields for the new PointCloud2
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        # Create new message with corrected header
        new_msg = point_cloud2.create_cloud(msg.header, fields, new_points)

        # FIX: Change frame_id from 'front_RPLidar' to 'chassis_link'
        new_msg.header.frame_id = 'chassis_link'

        # Publish
        self.publisher.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AddIntensityNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
