// cloud_generator.cpp
//
// Bill Smart
//
// This code generates fake point clouds of a specific size, to illustrate the latency behaviour of the
// ROS 2 middleware.


// Include the basic ROS functionality.
#include <rclcpp/rclcpp.hpp>

// Include the CloudGeneratorNode defintion.
#include <rob599_more_cpp/cloud_generator_node.hpp>


int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and give control to the ROS event handler.
	rclcpp::spin(std::make_shared<CloudGeneratorNode>());

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}