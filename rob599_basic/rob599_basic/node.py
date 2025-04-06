#!/usr/bin/env python3


# Example of a basic node in ROS 2.
#
# node.py
#
# Bill Smart
#
# This is the most basic of nodes.  It basically starts up a running node, then passes control
# to ROS.  It doesn't do anything useful.


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node


# The idiom in ROS2 is to encapsulate everything to do with the node in a class that
# is derived from the Node class in rclpy.node.
class BasicNode(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('useless_node')


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	node = BasicNode()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(node)

	# Make sure we shutdown everything cleanly.  This should happen even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a function as the node entry point and to call it from
	# the entry point of the script.
	main()
