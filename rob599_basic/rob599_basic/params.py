#!/usr/bin/env python3


# Example of parameters in a node.
#
# params.py
#
# Bill Smart
#
# This node shows how to use parameters in Python.


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# These are the imports for the parameter stuff.
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_event_handler import ParameterEventHandler

# We're going to publish an Int64.
from std_msgs.msg import Int64


# You guessed it, the idiom is still to inherit from Node.
class ParamDemo(Node):
	'''
	ROS parameter demo node.
	'''
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('param_demo')

		# You have to declare a parameter before you use it.
		self.declare_parameter('speed', 10.0)

		# Set up a callback to record the arrival of parameter changes.  First we
		# need a handler for the events.
		self.handler = ParameterEventHandler(self)

		# Now, we have to register a callback to receive.  You need to specify a 
		# parameter name and a node name, in addition to a callback.  The node
		# name does not have to be the same as the name of this node.  This means
		# that you can trigger the callback on parameter updates to other nodes.
		self.callback_handle = self.handler.add_parameter_callback(
			parameter_name = 'speed',
			node_name = 'param_demo',
			callback = self.parameter_cb
		)

		# Set up a timer to periodically print out the parameter value.
		self.timer = self.create_timer(1, self.timer_cb)

	def parameter_cb(self, parameter):
		value = parameter_value_to_python(parameter.value)
		self.get_logger().info(f'Parameter changed: {parameter.name} = {value}')

	# This callback will be called periodically.  Since on_callback_event() is not
	# implemented in Python, we're reduced to polling for the value.
	def timer_cb(self):
		# Extract the value of the parameter.
		speed = self.get_parameter('speed').get_parameter_value().double_value
		self.get_logger().info(f'Parameter value is: {speed}')


# This is the entry point
def main(args=None):
	# Initialize rclpy.
	rclpy.init(args=args)

	# Instantiate the node.
	demo = ParamDemo()

	# Give control to ROS.
	rclpy.spin(demo)

	# Make sure we shutdown everything cleanly.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	main()
