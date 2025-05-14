// cloud_shared.cpp
//
// Bill Smart
//
// An example of two nodes in one executable, to illustrate the effect on message latency.

// Include the basic ROS functionality.
#include <rclcpp/rclcpp.hpp>

// Include the definitions for the cloud generator and the cloud timer.
#include <rob599_more_cpp/cloud_generator_node.hpp>
#include <rob599_more_cpp/cloud_timer_node.hpp>


int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Make the nodes, and get shared pointers to them.  We can take advantage of the auto keyword
	// here, since the types are obvious.
	auto generator = std::make_shared<CloudGeneratorNode>();
	auto timer = std::make_shared<CloudTimerNode>();

	// Make an executor to take care of the callbacks and other ROS stuff.
	rclcpp::executors::MultiThreadedExecutor executor;

	// Add the nodes to the executor.
	executor.add_node(generator);
	executor.add_node(timer);

	// And, away we go!
	executor.spin();

	// Shut things down nicely.
	rclcpp::shutdown();

	return 0;
}