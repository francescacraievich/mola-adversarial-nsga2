#!/usr/bin/env python3
"""
MOLA Global Map Publisher Node

This node periodically saves the MOLA global map to a file and republishes it
as a PointCloud2 message for visualization in RViz.

The global map is persistent and accumulates all visited areas.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from mola_msgs.srv import MapSave
import struct
import numpy as np
import os
import tempfile
import time


class MOLAGlobalMapPublisher(Node):
    def __init__(self):
        super().__init__('mola_global_map_publisher')

        # Parameters
        self.declare_parameter('publish_rate', 2.0)  # Hz
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('save_directory', '/tmp/mola_maps')
        self.declare_parameter('downsample_points', 1)  # Publish every Nth point

        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_frame = self.get_parameter('map_frame').value
        self.save_dir = self.get_parameter('save_directory').value
        self.downsample = self.get_parameter('downsample_points').value

        # Create save directory
        os.makedirs(self.save_dir, exist_ok=True)

        # Publisher for global map
        self.map_publisher = self.create_publisher(
            PointCloud2,
            '/mola/global_map',
            10
        )

        # Service client to save MOLA map
        self.map_save_client = self.create_client(MapSave, '/map_save')

        # Timer to periodically save and publish map
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info(f'MOLA Global Map Publisher started')
        self.get_logger().info(f'Publishing rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Map frame: {self.map_frame}')
        self.get_logger().info(f'Save directory: {self.save_dir}')
        self.get_logger().info(f'Publishing on topic: /mola/global_map')

        self.map_saved_count = 0
        self.last_map_file = None

    def timer_callback(self):
        """Periodically save MOLA map and publish as PointCloud2"""

        # Wait for service
        if not self.map_save_client.wait_for_service(timeout_sec=0.5):
            if self.map_saved_count == 0:
                self.get_logger().warn('Waiting for /map_save service...')
            return

        # Save current map
        map_file = self.save_mola_map()

        if map_file is None:
            return

        self.last_map_file = map_file
        self.map_saved_count += 1

        # Try to load and publish the map
        # Note: This is a placeholder - actual loading depends on MOLA map format
        # For now, we'll publish a status message
        self.get_logger().info(
            f'Map saved successfully ({self.map_saved_count} times). '
            f'File: {map_file}'
        )

        # Create a simple status point cloud (since we can't easily parse .mm files in Python)
        self.publish_status_pointcloud()

    def save_mola_map(self):
        """Call MOLA map_save service"""

        # Generate unique filename
        timestamp = int(time.time() * 1000)
        map_path = os.path.join(self.save_dir, f'mola_map_{timestamp}.mm')

        # Create service request
        request = MapSave.Request()
        request.map_path = map_path

        # Call service synchronously
        try:
            future = self.map_save_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.result() is not None:
                response = future.result()
                if response.success:
                    return map_path
                else:
                    self.get_logger().error(f'Map save failed: {response.error_message}')
                    return None
            else:
                self.get_logger().error('Map save service call failed')
                return None

        except Exception as e:
            self.get_logger().error(f'Exception calling map_save: {str(e)}')
            return None

    def publish_status_pointcloud(self):
        """
        Publish a status point cloud.

        Note: Actual map publishing requires parsing MOLA .mm or .simplemap files,
        which needs MRPT libraries. For now, this publishes status info.
        """

        # Create a simple point cloud with status message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        # For demonstration: create a small point cloud
        # In a real implementation, you would load the actual map points
        points = np.array([
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ], dtype=np.float32)

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.is_dense = False
        msg.is_bigendian = False

        # Define fields
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width

        # Pack data
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', point[0], point[1], point[2]))

        msg.data = b''.join(buffer)

        # Publish
        self.map_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MOLAGlobalMapPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Total maps saved: {node.map_saved_count}')
        if node.last_map_file:
            node.get_logger().info(f'Last map file: {node.last_map_file}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
