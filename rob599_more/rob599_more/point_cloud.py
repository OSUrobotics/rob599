#!/usr/bin/env python3


# Point cloud examples.
#
# point_cloud.py
#
# Bill Smart


# Import ROS stuff.
import rclpy
from rclpy.node import Node

# We're going to be using Headers and PointCloud2s.
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

# There are some helper functions here.
from sensor_msgs_py import point_cloud2

# We'll fill the point clouds with random points.
from random import random


class PointCloudGenerator(Node):
	def __init__(self):
		super().__init__('point_cloud_generator')

		# Publish the point cloud on the cloud topic.
		self.pub = self.create_publisher(PointCloud2, 'cloud', 10)

		# Use a timer to periodically publish a point cloud.
		self.timer = self.create_timer(1, self.timer_callback)

	def timer_callback(self):
		# Generate the header.
		header = Header()
		header.stamp = self.get_clock().now().to_msg()
		header.frame_id = 'cloud_frame'

		# Generate the points.
		points = [(random(), random(), random()) for _ in range(5)]

		# Use the convenience function to make a point cloud of the right type.
		cloud = point_cloud2.create_cloud_xyz32(header, points)

		self.get_logger().info(f'Publishing a cloud with {len(points)} points.')

		# Publish the point cloud.
		self.pub.publish(cloud)


class PointCloudReader(Node):
	def __init__(self):
		super().__init__('point_cloud_reader')

		# Subscribe to the point cloud.
		self.sub = self.create_subscription(PointCloud2, 'cloud', self.callback, 10)

	def callback(self, msg):
		# Extract the points from the point cloud.
		points = point_cloud2.read_points(msg)

		# We're just going to print them out.
		self.get_logger().info(f'Got {len(points)} points.')
		for p in points:
			self.get_logger().info(f'-> {p}')



def generate(args=None):
	rclpy.init(args=args)

	generator = PointCloudGenerator()

	rclpy.spin(generator)

	rclpy.shutdown()


def read(args=None):
	rclpy.init(args=args)

	reader = PointCloudReader()

	rclpy.spin(reader)

	rclpy.shutdown()




if __name__ == '__main__':
	generator()
