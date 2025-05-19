#!/usr/bin/env python3

# filters.py
#
# Bill Smart
#
# Simple message filter example for ROS 2.

# Bring in the ROS stuff
import rclpy
from rclpy.node import Node

# Get the message_filters module.
import message_filters

# We're going to demonstrate this with fake LaserScans.
from sensor_msgs.msg import LaserScan


# Define a node.
class ApproxTimeFilterExample(Node):
	def __init__(self):
		super().__init__('filter_example')

		# We create two subscribers, using the subscriber call from the message_filters package.  If
		# we use the standard create_subscriber() function here, the message filters won't work.
		self.sub1 = message_filters.Subscriber(self, LaserScan, 'one')
		self.sub2 = message_filters.Subscriber(self, LaserScan, 'two')

		# Set up a time synchronizer for both publishers, queue size of 10, with a time window of
		# 0.01s.  Attach a callback that will trigger when two messages come in within this time of
		# each other.
		self.ts = message_filters.ApproximateTimeSynchronizer([self.sub1, self.sub2], queue_size=10, slop=0.01)
		self.ts.registerCallback(self.callback)

	# A callback with two parameters, one for each message.  The ordering of the messages is the same
	# as the lexical order in which they were passed to the ApproximateTimeSynchronizer constuctor,
	# above.
	def callback(self, msg_1, msg_2):
		self.get_logger().info(f'Got a pair of scans:\n  {msg_1.header.frame_id}: {msg_1.header.stamp}\n  {msg_2.header.frame_id}: {msg_2.header.stamp}')


# Entry point for the node.
def approx_time(args=None):
	# Initialize ROS.
	rclpy.init(args=args)

	# Instantiate a node.
	filter = ApproxTimeFilterExample()

	# Give control over to ROS.
	rclpy.spin(filter)

	# Shut things down cleanly.
	rclpy.shutdown()


if __name__ == '__main__':
	approx_time()
