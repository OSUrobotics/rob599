#!/usr/bin/env python3

# filter_publisher.py
#
# Bill Smart
#
# Publishes two fake LaserScans with similar timestamps to illustrate the use of
# message filters (in another node).

# Import ROS stuff.
import rclpy
from rclpy.node import Node

# We're going to use fake LaserScans.
from sensor_msgs.msg import LaserScan


# Define a node.
class FilterPublisher(Node):
	def __init__(self):
		super().__init__('filter_publisher')

		# Two publishers.
		self.pub_1 = self.create_publisher(LaserScan, 'one', 10)
		self.pub_2 = self.create_publisher(LaserScan, 'two', 10)

		# Two timers, one for each topic.
		self.timer_1 = self.create_timer(1, self.callback_1)
		self.timer_2 = self.create_timer(2, self.callback_2)

	# A callback for the first publisher.
	def callback_1(self):
		# Make a LaserScan, and set the timestamp and frame_id.
		laser = LaserScan()
		laser.header.stamp = self.get_clock().now().to_msg()
		laser.header.frame_id = 'laser_one'

		self.get_logger().info('Publishing for Laser 1')
		self.pub_1.publish(laser)


	# A callback for the second publisher.
	def callback_2(self):
		# Make a LaserScan, and set the timestamp and frame_id.
		laser = LaserScan()
		laser.header.stamp = self.get_clock().now().to_msg()
		laser.header.frame_id = 'laser_two'

		self.get_logger().info('Publishing for Laser 2')
		self.pub_2.publish(laser)


# Entry point for the node.
def main(args=None):
	# Initialize ROS.
	rclpy.init(args=args)

	# Instantiate a node.
	publisher = FilterPublisher()

	# Give control over to ROS.
	rclpy.spin(publisher)

	# Shut down cleanly.
	rclpy.shutdown()


if __name__ == '__main__':
	main()
