// action_client.cpp
//
// An example of a basic ROS 2 action client in C++.
//
// Bill Smart

// Include the basic ROS stuff.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// This is the action message definition.
#include <rob599_msgs/action/fibonacci.hpp>

// We'll need string streams to assemble some output and chrono literals to make things look nice.
#include <sstream>
#include <chrono>

// Use chrono literals.
using namespace std::chrono_literals;

// Placeholder arguments for callback binding.
using std::placeholders::_1;
using std::placeholders::_2;


// Make a node that inherits from the standard ROS Node, as usual.
class BasicActionClient : public rclcpp::Node {
public:
	// Define some type aliases to make the code look nicer.
	using Fibonacci = rob599_msgs::action::Fibonacci;
	using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

	// Constructor for the node.
	BasicActionClient() :Node("fibber_client") {
		// Create an action client.
		this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

		// Define a simple lambda function that sends the foal, and attach this to a timer.  This
		// will send the goal after a short wait, and will ensure that ROS is up and running before
		// the send happens (since the timer won't start up until then).
		auto timer_callback_lambda = [this](){return this->send_goal();};
		this->timer_ = this->create_wall_timer(500ms, timer_callback_lambda);
	}

private:
	// Variables to hold the action client and the timer.
	rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
	rclcpp::TimerBase::SharedPtr timer_;

	// The response callback.  This fires when the goal is accepted or rejected by the action server.
	// For now, we're just going to print out a message to the info logger.
	void response_callback(const GoalHandleFibonacci::SharedPtr &goal_handle) {
		if (!goal_handle) {
			RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
		} else {
			RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
		}
	}

	// The feedback callback.  This is called any time we get feedback from the action server.
	void feedback_callback(const GoalHandleFibonacci::SharedPtr &goal_handle, const std::shared_ptr<const Fibonacci::Feedback> &feedback) {
		// This will stop the compiler complaining about unused variables, since we're not going to
		// use the goal_handle in this example.
		(void)goal_handle;
		RCLCPP_INFO(this->get_logger(), "Feedback: %ld", feedback->progress);
	}

	// The result callback.  This gets called when the server sends back the result.
	void result_callback(const GoalHandleFibonacci::WrappedResult &result) {
		switch (result.code) {
			// Check the result code to see if things worked as expected, and print out a message.
			case rclcpp_action::ResultCode::SUCCEEDED:
				break;
			case rclcpp_action::ResultCode::ABORTED:
				RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
				return;
			case rclcpp_action::ResultCode::CANCELED:
				RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
				return;
			default:
				RCLCPP_ERROR(this->get_logger(), "Unknown result code");
				return;
		}

		// Make out output string stream, and populate it with the contents of the result.
		std::stringstream os;
		os << "Result received: ";
		for (auto number : result.result->sequence) {
			os << number << " ";
		}

		// Log the result.
		RCLCPP_INFO(this->get_logger(), os.str().c_str());

		// Shut down the node, since we're done.
		rclcpp::shutdown();
	}

	// This is a convenience function to send a goal to the action server.
	void send_goal() {
		// Since this is called automatically from the timer, cancel the timer.  If we don't do this,
		// the timer will periodically call send_goal(), which is not what we want.
	    this->timer_->cancel();

	    // Wait for the action server to start.  Print a message every second.
	    while (!this->client_ptr_->wait_for_action_server(1s))
			RCLCPP_INFO(this->get_logger(), "Waiting on action server to start");

		// Make a goal message type, and fill in the parameter.
		auto goal_msg = Fibonacci::Goal();
		goal_msg.number = 10;

		RCLCPP_INFO(this->get_logger(), "Sending goal");

		// We're going to put the callbacks into a structure to pass to ROS.
		auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

		// Register all of the callbacks in the SendGoalOptions structure.
		send_goal_options.goal_response_callback = std::bind(&BasicActionClient::response_callback, this, _1);
		send_goal_options.feedback_callback = std::bind(&BasicActionClient::feedback_callback, this, _1, _2);
		send_goal_options.result_callback = std::bind(&BasicActionClient::result_callback, this, _1);

		// Send the goal to the action server.
		client_ptr_->async_send_goal(goal_msg, send_goal_options);
	}
};


// This is the entry point for the node.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a client.
	auto client = std::make_shared<BasicActionClient>();

	// Give control over to ROS.
	rclcpp::spin(client);

	// Make sure everything is shut down cleanly.
	rclcpp::shutdown();

	// And, we're done.
	return 0;
}