// counter.cpp
//
// Bill Smart
//
// This is an example of a simple publisher in ROS 2.

// Include the basic ROS functionality.
#include <rclcpp/rclcpp.hpp>

// Include the definition of the Int64 message type.  The filename
// is package/msg/type.  Note that the type is all lower case in
// the filename.
#include <std_msgs/msg/int64.hpp>

// Include the ability to use chrono_literals, so that our code
// looks prettier.
#include <chrono>

// Use the namespace, so that we don't have to namespace the
// literals.
using namespace std::chrono_literals;


// Create a class that inherits from Node.
class BasicPublisher : public rclcpp::Node {
public:
	// Constructor.  Initialize the base class and a count variable on the initializer list.
	BasicPublisher() :Node("publisher"), count_(0) {
		// Create a publisher, with a topic name and publisher queue size.  The function is
		// templated and the type goes in the template.
		publisher_ = this->create_publisher<std_msgs::msg::Int64>("counter", 10);

		// Create a timer to control the publication.  This takes a time and a callback.  To
		// get the callback to work, we need to use std::bind, with a pointer to the callback
		// and a this reference.  The first argument uses chrono literals to make it more
		// readable for humans.
		timer_ = this->create_wall_timer(1s, std::bind(&BasicPublisher::timer_callback, this));
	}

private:
	// A variable to hold the current count.  The C++ idiom is to make private class variables
	// end in an underscore.
	long count_;

	// Variables for the publisher and timer.  These shoudl be shared pointers.
	rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// The callback that the timer uses.
	void timer_callback() {
		// Make a new message, and set the data element. We can use the auto
		// keyword here, since the type is clear.  Note that we're assigning
		// the data element and post-incrementing the count_ variable.
		auto message = std_msgs::msg::Int64();
		message.data = count_++;

		// Publish the message.
		publisher_->publish(message);

		// Record in the log that we published the message.
		RCLCPP_INFO(this->get_logger(), "Published %li", message.data);
	}
};


// This is the entry point of the executable.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and assign a shared pointer to it to the publisher variable.
	auto publisher = std::make_shared<BasicPublisher>();

	// Give control to the ROS event handler.
	rclcpp::spin(publisher);

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	return 0;
}
