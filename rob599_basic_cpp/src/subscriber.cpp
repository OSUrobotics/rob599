// subscriber.cpp
//
// Bill Smart
//
// An example of a C++ subscriber node.

// Include the basic ROS functionality and the message definition.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

// We need this in order to set up the subscriber.  This is the C++ way of
// letting us specify callback parameters.
using std::placeholders::_1;


// As usual, we create a class that inherits from Node.
class BasicSubscriber : public rclcpp::Node {
public:
	BasicSubscriber() :Node("subscriber") {
		// Set up the subscriber.  The function is templated and the message time goes in the
		// template specification.  The arguments are the topic name, queue size, and the
		// callback.  The callback argument uses std::bind to combine a pointer to the callback
		// and a placeholder for the single argument.
		subscriber_ = this->create_subscription<std_msgs::msg::Int64>("counter", 10, std::bind(&BasicSubscriber::callback, this, _1));
	}

private:
	// Variables for the subscriber.  The idiom is to end member variables with an underscore.
	rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;

	// The callback for the subscriber.  The message is passed via a shared pointer.  This means
	// you have to use the -> accessor, rather than the . (dot) accessor.
	void callback(const std_msgs::msg::Int64::SharedPtr msg) const {
		// Just announce that we got the message.
		RCLCPP_INFO(this->get_logger(), "Got %li", msg->data);
	}
};


// This is the entry point for the node.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node.
	auto subscriber = std::make_shared<BasicSubscriber>();

	// Give control to the ROS event handler.
	rclcpp::spin(subscriber);

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}