#!/usr/bin/env python3

# Example of modifying images with OpenCV and then republishing them.
#
# image_modifier.py
#
# Bill Smart


# Import what we need from ROS.
import rclpy
from rclpy.node import Node

# This is the basic image type
from sensor_msgs.msg import Image

# Import the OpenCV stuff
import cv2

# Import the OpenCV ROS bridge.
from cv_bridge import CvBridge


# Still the same idiom!
class ImageModifier(Node):
	def __init__(self):
		super().__init__('image_modifier')

		# Create an OpenCV bridge.  This will let us move between ROS and OpenCV images.
		self.bridge = CvBridge()

		# Subscribe to the incoming images.  Note that we're setting the buffer size to
		# 1.  If messages are coming in faster than we can process them, then we're going
		# to drop some of them.  This will mean that we're trying to keep up with the most
		# recent messags.
		self.sub = self.create_subscription(Image, 'image', self.modify_image, 1)

		# Publish the modified image stream.
		self.pub = self.create_publisher(Image, 'image_modified', 10)

	# This callback fires every time we get a new image.
	def modify_image(self, msg):
		# Convert from a ROS Image message to an OpenCV image.
		image = self.bridge.imgmsg_to_cv2(msg)

		# Do some processing on the OpenCV image.  In this case, we're just going to
		# do a simple blur.
		image = cv2.blur(image, (50, 50))

		# Convert back to the ROS Image message format.  We're going to copy the original
		# header information, and set the same encoding as the original image.
		new_msg = self.bridge.cv2_to_imgmsg(image)
		new_msg.header = msg.header
		new_msg.encoding = msg.encoding

		# Publish the new message.
		self.pub.publish(new_msg)


# Entry point for the node.
def main(args=None):
	# Initialize ROS.
	rclpy.init(args=args)

	# Set up the image modifier.
	modifier = ImageModifier()

	# Start everything up.
	rclpy.spin(modifier)

	# Make sure we shut down cleanly.
	rclpy.shutdown()


if __name__ == '__main__':
	# Call the entry point if we run this as a script.
	main()
