#!/usr/bin/env python3


# Example of directly writing to a ros2 bag.
#
# bag_writer.py
#
# Bill Smart
#
# This example directly writes to a ROS bag file using the API.


# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from rclpy.node import Node

# This is the module that interacts with ROS bags.
import rosbag2_py

# We're going to use the serializer to get things in the right format for the bag
# file
from rclpy.serialization import serialize_message

# We're going to write Strings to the bag.
from std_msgs.msg import String


# Inherit from the Node class
class BagWriter(Node):
	def __init__(self):
		# Initialize the parent class, giving it a name.  The idiom is to use the
		# super() class.
		super().__init__('bag_writer')

		# Specify a storage location, which can be a URI, and a type.  Generally,
		# use a file for the location, and 'mcap' the storage type.
		storage_options = rosbag2_py.StorageOptions(uri='example_bag', storage_id='mcap')

		# This allows for some data converstions.  We're not going to take advantage of
		# those for this example.
		converter_options = rosbag2_py.ConverterOptions('', '')

		# Create a writer object and open it for writing.
		self.writer = rosbag2_py.SequentialWriter()
		self.writer.open(storage_options, converter_options)

		# Define the topic information and send it to the writer.  We need an ID number,
		# a topic name and type, and a serialization format ('cdr' in this case).
		topic_info = rosbag2_py.TopicMetadata(
			id=0,
			name='chatter',
			type='std_msgs/msg/String',
			serialization_format='cdr',
		)
		self.writer.create_topic(topic_info)

	def write_text(self, text, offset):
		# Turn the text into a message.
		msg = String()
		msg.data = text

		# Calculate a fake timestamp offset from the current time.
		timestamp = offset * 1000000000 + self.get_clock().now().nanoseconds

		# Write to the bag file, with a timestamp.
		self.writer.write('chatter', serialize_message(msg), timestamp)


# This is the entry point
def main(args=None):
	# Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	# Make a bag writer.
	writer = BagWriter()

	for i in range(10):
		writer.write_text(f'Message {i}', i)

	# Make sure we shutdown everything cleanly.
	rclpy.shutdown()


# If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	main()
