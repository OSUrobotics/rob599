#!/usr/bin/env python3


# Example of publishing a Marker in ROS 2
#
# publisher.py
#
# Bill Smart
#
# This publishes a simple marker, moving in a circle.


# Include the ROS stuff.
import rclpy
from rclpy.node import Node

# Get the Marker type.
from visualization_msgs.msg import Marker

# Need this for the circle calculation
from math import sin, cos


# The idiom is still Node.
class MarkerPublisher(Node):
	def __init__(self):
		# Initialize the parent class.
		super().__init__('circle_marker')

		# Create a publisher for the marker.
		self.pub = self.create_publisher(Marker, 'dot', 10)

		# Set a timer to publish the marker.
		self.timer = self.create_timer(0.2, self.publish_marker)

	# This callback will be called every time the timer fires.
	def publish_marker(self):
		# Get the time, and convert it to seconds
		now = self.get_clock().now()
		time = now.nanoseconds / 1000000000

		# Make a Marker, and set the header information.  We have to convert the time, in the now
		# variable, to a message for this to work.
		marker = Marker()
		marker.header.frame_id = 'some_frame'
		marker.header.stamp = now.to_msg()

		# Set the ID, type, and action.
		marker.id = 0
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD

		# Set the pose
		marker.pose.position.x = cos(time)
		marker.pose.position.y = sin(time)
		marker.pose.position.z = 0.0

		# If we want an actual orientation, we need to use euler_to_quaternion and unpack it.  Since we
		# don't care, we can use a valid but fake quaternion.
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0

		# How big?
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1

		# What color?  This includes an alpha channel, a.  If this is set to 1.0, the color is opaque.  If
		# it's set to 0.0, then it's completely transparent (and, hence, invisible).  It will default to 0.0
		# because of the way variables are initialized.  Never forget to set this number.
		marker.color.r = 0.0
		marker.color.g = 1.0
		marker.color.b = 0.0
		marker.color.a = 1.0

		# And publish it.
		self.pub.publish(marker)

		self.get_logger().info(f'Published at ({marker.pose.position.x:.3f}, {marker.pose.position.y:.3f})')


# This is the entry point.
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a node class and call spin() on it.
	publisher = MarkerPublisher()
	rclpy.spin(publisher)

	# Make sure we shutdown everything cleanly.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	main()
