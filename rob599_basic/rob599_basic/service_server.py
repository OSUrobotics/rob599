#!/usr/bin/env python3


# Example of a service in ROS 2
#
# service.py
#
# Bill Smart
#
# This is an example of a simple service client in ROS 2.


# Pull in the stuff we need from rclpy.
import rclpy
from rclpy.node import Node

# Types for the service call.  We only need the base type, which corresponds to the
# Doubler.srv interface definition file.
from rob599_msgs.srv import Doubler


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
class BasicService(Node):
	def __init__(self):
		# Initialize the superclass.
		super().__init__('service')

		# Create a service, with a type, name, and callback.
		self.service = self.create_service(Doubler, 'doubler', self.callback)

	# This callback will be called every time that the service is called.  Both the
	# request and the response are passed in, avoiding the need to specify the response
	# type.
	def callback(self, request, response):
		# Fill in the data in the response type.
		response.doubled = request.number * 2

		# Log a message.
		self.get_logger().info(f'Got {request.number}')

		# The idiom is to return the response at the end of the callback.
		return response


# This is the entry point for the node.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# The ROS2 idiom is to encapsulate everything in a class derived from Node.
	service = BasicService()

	# Spin with the node, and explicily call shutdown() when we're done.
	rclpy.spin(service)
	rclpy.shutdown()


# This is the entry point when we call the node directly.
if __name__ == '__main__':
	main()
