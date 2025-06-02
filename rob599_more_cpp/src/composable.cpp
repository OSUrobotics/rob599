// composable.cpp
//
// Bill Smart
//
// Example of programmatic node composition in C++.


// Basic ROS stuff and messages.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>

// Time stuff.
#include <chrono>
using namespace std::chrono_literals;

// Parameter binding for callbacks.
using std::placeholders::_1;


// A simple publisher node.
class PublisherNode : public rclcpp::Node {
public:
	using Int64 = std_msgs::msg::Int64;

	PublisherNode() :Node("publisher"), count_(0) {
		// Make a publisher and a timer to trigger it.
		publisher_ = this->create_publisher<Int64>("counter", 10);
		timer_ = this->create_wall_timer(1s, std::bind(&PublisherNode::timer_callback, this));
	}

private:
	rclcpp::Publisher<Int64>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	int count_;

	void timer_callback() {
		// Make a message.
		auto message = Int64();
		message.data = count_++;

		// Publish the message.
		publisher_->publish(message);

		RCLCPP_INFO(this->get_logger(), "Published %li", message.data);
	}
};


// A simple subscriber node.
class SubscriberNode : public rclcpp::Node {
public:
	using Int64 = std_msgs::msg::Int64;

	SubscriberNode() :Node("subscriber") {
		subscriber_ = this->create_subscription<Int64>("counter", 10, std::bind(&SubscriberNode::callback, this, _1));

	}

private:
	rclcpp::Subscription<Int64>::SharedPtr subscriber_;

	void callback(const Int64::SharedPtr msg) const {
		// Just announce that we got the message.
		RCLCPP_INFO(this->get_logger(), "Got %li", msg->data);
	}
};


int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Instantiate the nodes.
	auto publisher = std::make_shared<PublisherNode>();
	auto subscriber = std::make_shared<SubscriberNode>();

	// Instantiate an executor and add the nodes.
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(publisher);
	executor.add_node(subscriber);

	// Give control to the executor.
	executor.spin();

	// Make sure we clean things up properly.
	rclcpp::shutdown();

	return 0;
}