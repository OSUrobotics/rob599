#!/usr/bin/env python3


# gemetric_cloud.py
#
# Bill Smart
#
# Generate and publish a point cloud with some geometric shapes in it.


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

# We'll need some math functions.
from math import pi, sin, cos


class GeometricCloudGenerator(Node):
	def __init__(self):
		super().__init__('geometric_cloud_generator')

		# Some parameters
		self.declare_parameter('noise', 0.01)
		self.declare_parameter('table_points', 100)
		self.declare_parameter('cylinder_points', 100)
		self.declare_parameter('noise_points', 100)

		# Publish the point cloud on the cloud topic.
		self.pub = self.create_publisher(PointCloud2, 'cloud', 10)

		# Use a timer to periodically publish a point cloud.
		self.timer = self.create_timer(1, self.timer_callback)

	def timer_callback(self):
		# Generate the header.
		header = Header()
		header.stamp = self.get_clock().now().to_msg()
		header.frame_id = 'map'

		# Get the current parameter values
		noise = self.get_parameter('noise').get_parameter_value().double_value
		table_points = self.get_parameter('table_points').get_parameter_value().integer_value
		cylinder_points = self.get_parameter('cylinder_points').get_parameter_value().integer_value
		noise_points = self.get_parameter('noise_points').get_parameter_value().integer_value

		# Generate the cloud.
		points = []

		# Generate the tabletop points.  The table is 2m by 3m, and 1m tall.
		points.extend([(random() * 3 - 1.5, random() * 2 - 1, 1) for _ in range(table_points)])

		# Generate the cylinder points.  The cylinder is 20cm in diameter and 40cm tall.
		angles = [random() * 2 * pi for _ in range(cylinder_points)]
		points.extend([(cos(a) * 0.1, sin(a) * 0.1, random() * 0.4 + 1) for a in angles])

		# Generate some noise points.
		points.extend([(random() * 4 - 2, random() * 3 - 1.5, random() * 1.6) for _ in range(noise_points)])

		# Add random noise to all the points.
		points = [(x + random() * 2 * noise - noise, y + random() * 2 * noise - noise, z + random() * 2 * noise - noise) for x, y, z in points]

		# Use the convenience function to make a point cloud of the right type.
		cloud = point_cloud2.create_cloud_xyz32(header, points)

		self.get_logger().info(f'Publishing a cloud with {len(points)} points.')

		# Publish the point cloud.
		self.pub.publish(cloud)


def main(args=None):
	rclpy.init(args=args)

	generator = GeometricCloudGenerator()

	rclpy.spin(generator)

	rclpy.shutdown()


if __name__ == '__main__':
	main()
