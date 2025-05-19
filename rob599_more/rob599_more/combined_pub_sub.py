#!/usr/bin/env python3

# combined_pub_sub.py
#
# Bill Smart
#
# An example of more than one node in a Python executable.

# Import the basic ROS functionality.
import rclpy

# Import the node classes.  This is one of reasons that inheriting from Node is useful.
from rob599_basic.publisher import BasicPublisher
from rob599_basic.subscriber import BasicSubscriber


# This is the entry point.
def main(args=None):
	# Initialize ROS.
	rclpy.init(args=args)

	# Create the nodes
	publisher = BasicPublisher()
	subscriber = BasicSubscriber()

	# Make an executor.  This deals with the callbacks and other ROS stuff.
	executor = rclpy.executors.MultiThreadedExecutor()

	# Add the nodes to the executor.
	executor.add_node(publisher)
	executor.add_node(subscriber)

	# And, away we go.
	executor.spin()

	# Shut down the node gracefully.
	rclpy.shutdown()


if __name__ == '__main__':
	main()