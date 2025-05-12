// Include the standard ROS stuff.
#include <rclcpp/rclcpp.hpp>

// This is the custom message that we're going to use.  Remember the naming convention:
// CamelCaseNames translate to camel_case_names.hpp
#include <rob599_msgs/srv/doubler.hpp>

// We want to use chrono literals, since they make our code look nicer.
#include <chrono>
using namespace std::chrono_literals;


// Define the node.
class BasicServiceClient : public rclcpp::Node {
public:
	// Use an alias to simplify the syntax.
	using Doubler = rob599_msgs::srv::Doubler;

	BasicServiceClient() :Node("client") {
		// Create a service client.
		client_ = this->create_client<Doubler>("doubler");

		// Wait until the service server is ready.  Timeout every 1s and print out an
		// and appropriate message.  The 1s argument is a chrono literal.
		while (!client_->wait_for_service(1s)) {
			// Check to see if the node got shut down.  If so, quit the constructor.
			if (!rclcpp::ok()) {
				RCLCPP_INFO(this->get_logger(), "Shut down while waiting for the service.");
				break;
			}
			RCLCPP_INFO(this->get_logger(), "waiting for service to start");
		}
	}

	// Send an asynchronous request, and return a future to it.  We're going to cheat slightly
	// and use auto to calcuate the return type here, since the compiler can figure it out and
	// it's a pretty gnarly compound type.  This is really just a convenience function that wraps
	// up the creation and population of the Request message type.
	auto send_request(const int number) {
		// Make the request and fill in the fields.
		auto request = std::make_shared<Doubler::Request>();
		request->number = number;

		// Make the async call and return the future.
		return client_->async_send_request(request);
	}

private:
	// A variable to hold the pointer to the service client.
	rclcpp::Client<Doubler>::SharedPtr client_;
};


// This is the entry point of the node.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Make a node and get a shared pointer to it.
	auto client = std::make_shared<BasicServiceClient>();

	// Loop to show a sequence of calls.
	for(int i = 0; i < 5; ++i) {
		// Make the call and collect the future.
		auto future = client->send_request(i);

		// This function spins until the future is ready, then returns a whether or not the call was
		// successful.  If it was, and returned SUCCESS, then we extract the result.  If not, we print
		// an error message and move onto the next value.
		if (rclcpp::spin_until_future_complete(client, future) == rclcpp::FutureReturnCode::SUCCESS)
			RCLCPP_INFO(client->get_logger(), "%i -> %ld", i, future.get()->doubled);
		else
			RCLCPP_WARN(client->get_logger(), "service call failed for %i", i);
	}

	rclcpp::shutdown();
}