#!/usr/bin/env python3

# cloud_generator.py
#
# Bill Smart
#
# Node to generate PointCloud2 messages to test latency when compared to C++ implementation.

# Import the ROS stuff
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2


# A basic publishing class.
class CloudGenerator(Node):
	def __init__(self):
		super().__init__('cloud_generator')

		self.pub = self.create_publisher(PointCloud2, 'clouds', 10)
		self.timer = self.create_timer(0.25, self.timer_callback)

	def timer_callback(self):
		cloud_size = 640 * 480

		# Build a message.  This is to compare to the C++ implementation, compare the latencies, so
		# we're just going to do a basic implementation.
		message = PointCloud2()
		message.height = 1
		message.point_step = 1
		message.is_bigendian = False
		message.is_dense = True
		message.width = cloud_size
		message.row_step = cloud_size
		message.data = [0] * cloud_size

		message.header.stamp = self.get_clock().now().to_msg()

		self.pub.publish(message)


def main(args=None):
	# Initialze ROS.
	rclpy.init(args=args)

	# Make a node instance.
	cloud_generator = CloudGenerator()

	# Give control to ROS.
	rclpy.spin(cloud_generator)

	# Make sure we're shutdown properly.
	rclpy.shutdown()


if __name__ == '__main__':
	main()
