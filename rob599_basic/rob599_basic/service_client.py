#!/usr/bin/env python3

# Example of a service client in ROS 2
#
# service_client.py
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
class BasicServiceClient(Node):
	def __init__(self):
		# Initialize the superclass.
		super().__init__('client')

		# Set up a service client with a type and a service name.
		self.client = self.create_client(Doubler, 'doubler')

		# Wait until we have a connection to the server.
		while not self.client.wait_for_service(timeout_sec=1):
			self.get_logger().info('waiting for service to start')

	# This function will actually make the call for us.  This a convenience function
	# to hide some of the plumbing in the class, so that the service all is easier to
	# use.
	def send_request(self, number):
		# Build a request.  Request types now are attributes of the main type, and are
		# retrieved with Request().  Fill in the fields once we have it.
		request = Doubler.Request()
		request.number = number

		# Service calls are now aynchronous by default.  We're going to store the result
		# of the call in an instance variable.
		self.response = self.client.call_async(request)


# This is the entry point of the node.
def main(args=None):	
	# Initialize rclpy.
	rclpy.init(args=args)

	# Get a node-derived class.
	client = BasicServiceClient()

	# We're going to make five calls to the service.
	for i in range(5):
		# Make the asynchronous call.
		client.send_request(i)

		# We're going to loop until we get a result.  This will let us abandon things if the
		# server is offline.
		while rclpy.ok():
			# Let rclpy do things behind the scenes.  In ROS2, this needs a node, just like
			# spin() does.
			rclpy.spin_once(client)

			# If we've had a response, try to retrieve it.
			if client.response.done():
				try:
					# We get the response through result()
					answer = client.response.result()
				except Exception as e:
					# An exception will be thrown if things fail.
					client.get_logger().info('Service call failed: {0}'.format(e))
				else:
					# Then, we're going to log the result.
					client.get_logger().info('Send {0}, got {1}'.format(i, answer.doubled))
					break;

	# Shut things down when we're done.
	rclpy.shutdown()


# This is the entry point if we run this node as an executable.
if __name__ == '__main__':
	main()
