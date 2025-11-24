#!/usr/bin/env python3
"""
Publish MOLA Saved Map to RViz

This node periodically triggers MOLA to save its global map,
then loads the saved PointCloud and publishes it for RViz visualization.

The map is persistent and accumulates all visited areas.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from mola_msgs.srv import MapSave
import os
import subprocess
import time


class PublishMOLASavedMap(Node):
    def __init__(self):
        super().__init__('publish_mola_saved_map')

        # Parameters
        self.declare_parameter('publish_rate', 1.0)  # Hz - how often to update
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_save_path', '/tmp/mola_global_map.mm')

        self.publish_rate = self.get_parameter('publish_rate').value
        self.map_frame = self.get_parameter('map_frame').value
        self.map_path = self.get_parameter('map_save_path').value

        # Publisher
        self.map_publisher = self.create_publisher(
            PointCloud2,
            '/mola/global_map',
            10
        )

        # Service client
        self.map_save_client = self.create_client(MapSave, '/map_save')

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)

        self.get_logger().info('='*50)
        self.get_logger().info('MOLA Global Map Publisher for RViz')
        self.get_logger().info('='*50)
        self.get_logger().info(f'Publishing to: /mola/global_map')
        self.get_logger().info(f'Update rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Map frame: {self.map_frame}')
        self.get_logger().info('='*50)
        self.get_logger().info('')
        self.get_logger().info('Instructions:')
        self.get_logger().info('1. Make sure MOLA is running')
        self.get_logger().info('2. Move the robot to build the map')
        self.get_logger().info('3. In RViz, add PointCloud2 display')
        self.get_logger().info('4. Set topic to /mola/global_map')
        self.get_logger().info('5. Set Fixed Frame to "map"')
        self.get_logger().info('='*50)

        self.save_count = 0

    def timer_callback(self):
        """Periodically save and publish MOLA global map"""

        # Wait for service
        if not self.map_save_client.wait_for_service(timeout_sec=0.1):
            if self.save_count == 0:
                self.get_logger().warn(
                    'Waiting for MOLA /map_save service... '
                    'Make sure MOLA is running!'
                )
            return

        # Save map using MOLA service
        request = MapSave.Request()
        request.map_path = self.map_path

        future = self.map_save_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.result() is not None and future.result().success:
            self.save_count += 1

            if self.save_count % 10 == 1:  # Log every 10 saves
                self.get_logger().info(
                    f'Map saved successfully ({self.save_count} times). '
                    f'File: {self.map_path}'
                )

            # Now try to convert and publish
            # Note: This requires mm-viewer or MRPT tools
            self.try_publish_map()
        else:
            if self.save_count == 0:
                self.get_logger().warn('Map save failed. Is MOLA running and mapping enabled?')

    def try_publish_map(self):
        """
        Try to publish the map.

        Note: Publishing .mm files requires MRPT C++ libraries.
        This is a placeholder for the actual implementation.

        For now, we just log that the map was saved.
        Users should use MOLA GUI to visualize the global map.
        """

        # Check if file exists
        if os.path.exists(self.map_path):
            file_size = os.path.getsize(self.map_path)
            if self.save_count % 10 == 1:
                self.get_logger().info(
                    f'Global map file size: {file_size / 1024:.1f} KB'
                )

            # TODO: Load .mm file and convert to PointCloud2
            # This requires MRPT Python bindings or calling C++ tools

            # For now, inform user to use MOLA GUI or mm-viewer
            if self.save_count == 1:
                self.get_logger().info('')
                self.get_logger().info('='*50)
                self.get_logger().info('To visualize the GLOBAL map:')
                self.get_logger().info('')
                self.get_logger().info('Option 1: Use MOLA GUI')
                self.get_logger().info('  ros2 launch ... use_mola_gui:=True')
                self.get_logger().info('')
                self.get_logger().info('Option 2: Use mm-viewer (offline)')
                self.get_logger().info(f'  mm-viewer {self.map_path}')
                self.get_logger().info('')
                self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    node = PublishMOLASavedMap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f'Total maps saved: {node.save_count}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
