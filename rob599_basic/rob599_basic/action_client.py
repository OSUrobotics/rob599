#!/usr/bin/env python3


# Example of an action client in ROS 2
#
# action_client.py
#
# Bill Smart
#
# This is an example of a action client in ROS 2.


# Pull in the stuff we need from rclpy.
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient


# Pull in the action definition.  As with other compound messages, we only need to bring
# in the base type, since all of the other types are defined inside of it.
from rob599_msgs.action import Fibonacci


# The idiom in ROS2 is to use a function to do all of the setup and work.  This
# function is referenced in the setup.py file as the entry point of the node when
# we're running the node with ros2 run.  The function should have one argument, for
# passing command line arguments, and it should default to None.
class FibonacciClient(Node):
	def __init__(self, with_cancel=False):
		# Initialize the superclass
		super().__init__('fib_client')

		# Set up the simple action client.  This is a different procedure from topics and
		# services, since actions are not part of the Node class.  However, we're still
		# going to pass in a node (as self), a type and an action name.
		self.client = ActionClient(self, Fibonacci, 'fibonacci')

		# Are we going to show cancelation functionality?
		self.with_cancel = with_cancel

	# This function is a wrapper that will allow us to more conveniently invoke the action.
	def send_goal(self, n):
		# Build an action goal, and fill in the data.  As with the other compound message
		# types, we can get to this through the base type.
		goal = Fibonacci.Goal()
		goal.number = n

		# Wait until the server is ready to accept an action request.
		self.client.wait_for_server()

		# Make the action request.  We're going to send the goal message, and set the
		# feedback callback.  We're going to store the handle in an instance variable.
		self.result = self.client.send_goal_async(goal, feedback_callback=self.feedback_cb)

		# Attach a callback to the call, so that we can react when the action is accepted or
		# rejected.  The name "done" is a bit misleading, since the action is not done when
		# this callback is fired.  The callback is called when the action is accepted or
		# rejected.
		self.result.add_done_callback(self.response)

	# Process feedback as it comes in.
	def feedback_cb(self, feedback_msg):
		# We're not going to do anything other than log the feedback.
		self.get_logger().info(f'Got feedback: {feedback_msg.feedback.progress}')

		# This will test the functionality of the cancelation request.
		if self.with_cancel and feedback_msg.feedback.progress == 7:
			self.get_logger().info('7 is enough.  Canceling request.')
			future = self.goal_handle.cancel_goal_async()
			future.add_done_callback(self.cancel_cb)

	def cancel_cb(self, future):
		response = future.result()

		if len(response.goals_canceling) > 0:
			self.get_logger().info('Goal canceled.')
		else:
			self.get_logger().info('Goal failed to cancel.')			

	# This callback fires when the action is accepted or rejected.
	def response(self, future):
		# Get the result of requesting the action.  Note that this is not the result of
		# performing the action.  This just tells us if the action server accepted the
		# request and is starting to work on the action.
		self.goal_handle = future.result()

		# If it's not accepted, then we're done.  Log a message and return from the
		# function.
		if not self.goal_handle.accepted:
			self.get_logger().info('Goal rejected')
			return

		# If the action was accepted, then get a handle to it, so that we can retrieve the
		# result.  Then, associate a callback with it, so that we get access to the results
		# when they're available.
		self.result_handle = self.goal_handle.get_result_async()
		self.result_handle.add_done_callback(self.process_result)

	# This callback fires when there are results to be had.  This happens when the action
	# is finished.
	def process_result(self, future):
		# Get the result.  This has a type corresponding to the result part of the
		# action message definition.
		result = future.result().result

		# Log the result to the info channel.
		self.get_logger().info(f'Result: {list(result.sequence)}')


# This is the main entry point.
def main(client, args=None):
	# Initialize rclpy.
	rclpy.init(args=args)



def without_cancel(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Set up a node to do the work.
	client = FibonacciClient(with_cancel=False)

	# Make the action call.
	client.send_goal(10)

	# Give control over to ROS2.
	rclpy.spin(client)

	# Make sure everything has shut down correctly.
	rclpy.shutdown()
	

def with_cancel(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Set up a node to do the work, demonstrating action canceling.
	client = FibonacciClient(with_cancel=True)

	# Make the action call.
	client.send_goal(10)

	# Give control over to ROS2.
	rclpy.spin(client)

	# Make sure everything has shut down correctly.
	rclpy.shutdown()
	
	

# This is the entry point for running the node directly from the command line.
if __name__ == '__main__':
	main()
