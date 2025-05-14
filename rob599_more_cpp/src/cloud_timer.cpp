// cloud_timer.cpp
//
// Bill Smart
//
// This node receives PointCloud2 messages, and calculates the latency based on the arrival time
// and the message timestamp.

// Get the basic ROS functionality, and the definitions of the types we need.
#include <rclcpp/rclcpp.hpp>

// The cloud timer code is actually defined in this header file.
#include <rob599_more_cpp/cloud_timer_node.hpp>


int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and give control to the ROS event handler.
	rclcpp::spin(std::make_shared<CloudTimerNode>());

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}