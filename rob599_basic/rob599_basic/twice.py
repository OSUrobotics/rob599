#!/usr/bin/env python3


# Example of a simple transformer node in ROS 2
#
# twice.py
#
# Bill Smart
#
# This node subscribes to a topic with Int64s.  It multiplies the numbers by a constant,
# and then republishes them on another topic.


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# We're going to publish an Int64.  This is the same as ROS.
from std_msgs.msg import Int64


class Twice(Node):
	'''
	This node subscribes to a topic publishing Int64s, doubles each number if receives, and
	# repubishes the result on an outbound topic.
	'''
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('twice')

		# Create the publisher first, to make sure it's available before we start the subscriber.
		self.pub = self.create_publisher(Int64, 'doubled', 10)

		# Create the subscriber.
		self.sub = self.create_subscription(Int64, 'number', self.callback, 10)


	# This callback will be called whenever we receive a new message on the topic.
	def callback(self, msg):
		# Create a new message.
		new_msg = Int64()

		# Multiply the data element of the message.
		new_msg.data = msg.data * 2

		# Make sure we record what's going on
		self.get_logger().info(f'Got {msg.data} and transformed it to {new_msg.data}')

		# Republish the message.  Reusing the original message is slightly more efficient
		# than creating a new one.
		self.pub.publish(new_msg)


# This is a entry point.	
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class.  The idiom in ROS2 is to encapsulte everything in a class
	# that derives from Node.
	twice = Twice()

	# The spin() call gives control over to ROS2, and it now takes a Node-derived
	# class as a parameter.
	rclpy.spin(twice)

	# Make sure we shutdown everything cleanly.  This should happen, even if we don't
	# include this line, but you should do it anyway.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	# The idiom in ROS2 is to set up a main() function and to call it from the entry
	# point of the script.
	twice()
