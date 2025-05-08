#!/usr/bin/env python3

# Example of coordinating a set of lifecycle nodes.
#
# coordinator.py
#
# Bill Smart


# Import the usual ROS stuff
import rclpy
from rclpy.node import Node

# We need the ChangeState service type declaration and the Transition message
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition


# Make a node to hold the service client and wrap up the state change request.
# We're going to pass in the lifecycle node name as a parameter.
class LifecycleCoordinator(Node):
	def __init__(self, node_names):
		super().__init__('coordinator')

		# Make a service clients for each of the nodes we want to coordinate.
		self.client_list = [self.create_client(ChangeState, f'{name}/change_state') for name in node_names]

		# Wait until the service is available and the client is connected.
		for client in self.client_list:
			while not client.wait_for_service(timeout_sec=1):
				self.get_logger().info('waiting for lifecycle node to become available')

	# This wraps up the state change request.
	def change_state(self, state):
		request = ChangeState.Request()
		request.transition.id = state

		# Make the calls, one to each node, and store the futures in a list.
		self.responses = [client.call_async(request) for client in self.client_list]

	# Are all of the responses done?  If any of them are not done, then return False.
	# Otherwise, return True.
	def requests_done(self):
		for response in self.responses:
			if not response.done():
				return False

		return True


# This is the entry point of the node.
def main(args=None):
	# Initialize ROS.
	rclpy.init(args=args)

	# A list of the nodes we want to coordinate.
	node_names = ['/counter_1', '/counter_2']

	# Create the manager node, parameterized by the node name of the lifecycle node.
	manager = LifecycleCoordinator(node_names)

	# We're going to go through a fixed set of states for this example.  In this example, we're
	# going to assume each node has just started.  In a more robust implementation, we would first
	# check the state of each node, moving it to the desired start state.  This can be done through
	# the lifecycle node service interface.
	state_sequence = [
		Transition.TRANSITION_CONFIGURE,
		Transition.TRANSITION_ACTIVATE,
		Transition.TRANSITION_DEACTIVATE,
		Transition.TRANSITION_ACTIVATE,
		Transition.TRANSITION_DEACTIVATE,
		Transition.TRANSITION_CLEANUP,
		Transition.TRANSITION_UNCONFIGURED_SHUTDOWN,
	]

	# Go through the states one at a time.
	for state in state_sequence:
		# Make the state change request.  If we're coorindating a number of states, then we would
		# make a call like this to each node, and would synchronize the results from them.
		manager.get_logger().info('Requesting state change')
		manager.change_state(state)

		# Wait until the future is done.  Quit the loop if ROS has shutdown.  Don't loop if we've
		# already processed the response.  If we don't have this last condition, we will sometimes
		# process the return twice, since the enclosing for loops fast enough that we get to the 
		# while again before the new call has filled in the future correctly.  We could also solve
		# this by putting a short sleep after the change_state call above, to allow time for the
		# future to get updates.  Using the explicit processed variable is a better solution, since
		# it's explicit what we're doing.
		processed = False
		while not manager.requests_done() and rclpy.ok() and not processed:
			# Make sure ROS is processing.
			rclpy.spin_once(manager)

			try:
				# Extract the result from the future.
				processed = True
				results = [response.result() for response in manager.responses]
			except Exception as e:
				manager.get_logger().error(f'  failed: {e}')
			else:
				manager.get_logger().info(f'  successful: {state}')

		# Wait for someone to press the enter key.
		input('Waiting...')


if __name__ == '__main__':
	main()
