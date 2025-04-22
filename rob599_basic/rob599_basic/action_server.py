#!/usr/bin/env python3

# Example of an action server in ROS 2
#
# action_server.py
#
# Bill Smart
#
# This is an example of a action server in ROS 2.


# Pull in the stuff we need from rclpy.
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse

# These are needed if we're going to enable action cancelation.
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Pull in the action definition.  As with other compound messages, we only need to bring
# in the base type, since all of the other types are defined inside of it.
from rob599_msgs.action import Fibonacci

# We're going to use sleep to slow down the action server.  This will let us see the
# feedback coming into the client.
from time import sleep


# We're going to use this as the main worker function for the action server.  We've
# intentionally added a delay to this, so that it runs slowly.  This will let us see
# the feedback results in the action client.  In general, though, this would be a 
# dumb thing to do.
def fibonacci(n):
	"""
	A naive implementation of Fibonacci numbers.

	:param n: An integer.
	:return: The nth Fibonacci number.
	"""
	if n < 2:
		# Wait for a little bit so that we can see the progress in the client.
		sleep(0.01)

		return n
	else:
		return fibonacci(n - 1) + fibonacci(n - 2)


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
class FibonacciActionServer(Node):
	def __init__(self):
		# Initialize the superclass
		super().__init__('fibber')

		# Set up a simple action server.  This is a different procedure than for topics
		# and services, since actions are still not really first-class citizens.  However,
		# we're still going to pass in the node (as self), in addition to a type, an
		# action name, and a callback.
		# If we're going to enable action cancellation, we need the callback_group and
		# cancel_callback arguments.  If we don't want to cancel actions, then we can
		# omit these.
		self.server = ActionServer(self, Fibonacci, 'fibonacci', self.callback, 
			callback_group=ReentrantCallbackGroup(), cancel_callback=self.cancel_callback)

	# This is the callback the services the action request.
	def callback(self, goal):
		# Grab the logger and send a message to it.
		self.get_logger().info(f'Got {goal.request.number}')

		# Build a result to send the sequence to.  We can do this through the base
		# action type.  Once we build it, we set the return data, in sequence, to an
		# empty list.
		result = Fibonacci.Result()
		result.sequence = []

		# Incrementally fill in the elements of the list by making calls to the naive
		# Fibonacci number generator.
		for i in range(goal.request.number + 1):
			# Check to see if we have a cancellation request.  If we do, set the goal
			# status to canceled and return an empty result.
			if goal.is_cancel_requested:
				goal.canceled()
				self.get_logger().info('And, the goal is canceled.')
				return Fibonacci.Result()

			result.sequence.append(fibonacci(i))
			goal.publish_feedback(Fibonacci.Feedback(progress=i))

			self.get_logger().info(f'Partial result: {list(result.sequence)}')

		# Let the action server know that we've succeeded in the action.  It it doesn't
		# succeed, you can set other values here.
		goal.succeed()
		self.get_logger().info(f'Result: {list(result.sequence)}')

		# Return the result to the action server.
		return result

	# This callback fires when a cancellation request comes in.
	def cancel_callback(self, goal_handle):
		self.get_logger().info('Canceling goal')
		return CancelResponse.ACCEPT


# This is the entry point.
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Set up a node to do the work.
	server = FibonacciActionServer()

	# Give control over to ROS2.  To make the goal cancelation work, we need a
	# multithreaded executor here.  If we don't need goal cancelation, we can
	# use the default executor.
	rclpy.spin(server, MultiThreadedExecutor())

	# Make sure we shut down politely.
	rclpy.shutdown()


# This is the entry point for running the node directly from the command line.
if __name__ == '__main__':
	main()
