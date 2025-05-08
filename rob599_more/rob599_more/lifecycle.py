#!/use/bin/env python3


# An example of lifecycle nodes in Python
#
# lifecycle.py
#
# Bill Smart


# Import the basic ROS stuff.
import rclpy

# Import the lifecycle node stuff.  We need LifecycleNode, which acts just like
# rclpy.Node, and TransitionCallbackReturn, which contains return codes for the
# callbacks that implement the lifecycle.
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn

# We're going to publish Int64 messages, as usual.
from std_msgs.msg import Int64


# Create a class that inherits from LifecycleNode.
class LifecycleExample(LifecycleNode):
	def __init__(self):
		# Initialize the parent node, as usual, passing the node name.
		super().__init__('lifecycle_example')

	# Configure your node in this callback, which is called when the node enters the
	# configure state.  You should create publishers, subscribers, service clients,
	# action_clients, timers, and anything else that will take a while to set up in
	# this callback.
	def on_configure(self, previous_state):
		self.get_logger().info('Configuring')

		# Create a counter.
		self.counter = 0

		# Create a publisher.  We're going to create a lifecycle publisher, which will
		# not publish anything unless the node is in the active state.  The parameters
		# are the same as usual.  We're not going to start publishing until we're active
		# in any case, but it's good practice to use this version in case we get things
		# wrong elsewhere in the code.
		self.pub = self.create_lifecycle_publisher(Int64, 'counter', 10)

		# Create a timer for the callback, but make sure it doesn't start by using the
		# autostart parameter.
		self.timer = self.create_timer(0.5, self.timer_callback, autostart=False)

		# Let ROS know that we successfully completed the configuration.
		return TransitionCallbackReturn.SUCCESS

	# Start your node doing work in this callback, which is called when the node enters
	# the active state.  This should start anything that was set up, but not started, in
	# the on_configure callback.
	def on_activate(self, previous_state):
		self.get_logger().info('Activating')

		# Start the timer.  This will start calling the publishing callback.
		self.timer.reset()

		# Call the on_activate callback from the LifecycleNode parent class.  This will
		# start any publishers created by create_lifecycle_publisher, which are not
		# enabled unless the node is in the active state.  If we just return SUCCESS from
		# this callback, publishers created with create_lifecycle_publisher will not
		# publish anything.
		return super().on_activate(previous_state)
	
	# Suspend the node in this callback, which is called when the node enters the deactivated
	# state.  Don't get rid of anything here, just suspend it.
	def on_deactivate(self, previous_state):
		self.get_logger().info('Deactivating')

		# Suspend the timer, which will prevent the publishing callback from being called.
		# This does not get rid of the timer, it just pauses it.
		self.timer.cancel()

		# Call the deactivate callback from the LifecycleNode parent class.  This will
		# stop any publishers created by create_lifecycle_publisher.  If we just return
		# SUCCESS from this callback, publishers created with create_lifecycle_publisher
		# will continue publishing if we make a publish() call on them.
		return super().on_deactivate(previous_state)
		
	# Get rid of any resources we allocated in the on_configure callback.
	def on_cleanup(self, previous_state):
		self.get_logger().info('Cleaning up')

		# Get rid of the timer.
		self.destroy_timer(self.timer)

		# Get rid of the publisher.
		self.destroy_lifecycle_publisher(self.pub)

		# Let ROS know that we successfully completed the cleanup.
		return TransitionCallbackReturn.SUCCESS
		
	# Perform any shutdown actions you need to do for the node.  These should only be carried
	# out once, at the end of the useful life of the node.
	def on_shutdown(self, previous_state):
		self.get_logger().info('Shutting down')

		# Get rid of the timer.
		self.destroy_timer(self.timer)

		# Get rid of the publisher.
		self.destroy_lifecycle_publisher(self.pub)

		# Let ROS know that we successfully completed the shutdown.
		return TransitionCallbackReturn.SUCCESS
	
	# If there's an error in any of the previous states, this callback will be called.  You
	# should deal with any problems here, clean up any resources you need to, and make sure
	# things are in a safe state on your robot.
	def on_error(self, previous_state):
		self.get_logger().info('ERROR!')

		return TransitionCallbackReturn.ERROR

	# Simple timer to publish the Int64 message.
	def timer_callback(self):
		# Make the message and populate it.
		msg = Int64()
		msg.data = self.counter

		# Publish the message.
		self.pub.publish(msg)

		self.get_logger().info(f'  published {msg.data}')

		# Increment the counter
		self.counter += 1


def main(args=None):
	# Initialize ROS.
	rclpy.init(args=args)

	# Make an instance of the node.
	node = LifecycleExample()

	# Start up ROS.
	rclpy.spin(node)

	# Make sure things are shutdown cleanly.
	rclpy.shutdown()


if __name__ == '__main__':
	main()
