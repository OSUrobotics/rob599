#!/usr/bin/env python3

# latching.py
#
# Bill Smart
#
# Latching topic example.


# Import the ROS stuff.
import rclpy
from rclpy.node import Node

# Message type
from std_msgs.msg import Int64

# We're going to use these to modify the middleware Quality-of-Service (QoS) parameters.
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


# Define a publisher node.
class LatchingPublisher(Node):
	def __init__(self):
		super().__init__('latching_publisher')

		self.counter = 0

		# Define the QoS profile.  This profile is the one used for latched topics.  It's
		# reliable, retains the single last message, and is TRANSIENT_LOCAL.
		qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.RELIABLE,
			history=QoSHistoryPolicy.KEEP_LAST,
			depth=1,
			durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
		)

		# Create to publisher with the defined QoS.  This takes the place of the queue
		# size in previous calls.  What we've been thinking of as a queue size, is really
		# a QoS setting that is translated under the hood by the create_publisher call.
		self.pub = self.create_publisher(Int64, 'counter', qos_profile)

		# A timer for the callback.
		self.timer = self.create_timer(5, self.timer_callback)

	# A simple callback that publishes the counter value.
	def timer_callback(self):
		msg = Int64()
		msg.data = self.counter

		self.pub.publish(msg)
		self.get_logger().info(f'Published {msg.data}')

		self.counter += 1


# Define the subscriber node.  It has one parameter, whether it is latching or not.
class LatchingSubscriber(Node):
	def __init__(self, latching=False):
		super().__init__('latched_subscriber')

		# If we're latching, then set the QoS.  If not, set the qos_profile to 10, which
		# will be interpreted correctly by the create_subscription call.
		if latching:
			qos_profile = QoSProfile(
				reliability=QoSReliabilityPolicy.RELIABLE,
				history=QoSHistoryPolicy.KEEP_LAST,
				depth=1,
				durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
			)
		else:
			qos_profile = 10

		# Create the subscriber with the correct QoS.
		self.sub = self.create_subscription(Int64, 'counter', self.callback, qos_profile)

	# Just print out the value when we get it.
	def callback(self, msg):
		self.get_logger().info(f'Received {msg.data}')


# Entry point.  You know the drill by now.
def latching_publisher(args=None):
	rclpy.init(args=args)

	publisher = LatchingPublisher()

	rclpy.spin(publisher)

	rclpy.shutdown()


# Entry point.
def latching_subscriber(args=None):
	rclpy.init(args=args)

	subscriber = LatchingSubscriber(True)

	rclpy.spin(subscriber)

	rclpy.shutdown()


# Entry point.
def subscriber(args=None):
	rclpy.init(args=args)

	subscriber = LatchingSubscriber(False)

	rclpy.spin(subscriber)

	rclpy.shutdown()


if __name__ == '__main__':
	latching_publisher()