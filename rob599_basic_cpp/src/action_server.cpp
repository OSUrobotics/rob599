// action_client.cpp
//
// An example of a basic ROS 2 action client in C++.
//
// Bill Smart


// Include the basic ROS stuff.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// This is the action message type.
#include <rob599_msgs/action/fibonacci.hpp>

// We're going to use string streams to assemble log messages, and threads for the servicing of the
// action request.
#include <sstream>
#include <thread>


// Placeholders for binding the callback functions.
using std::placeholders::_1;
using std::placeholders::_2;


// A terrible function to calculate Fibonacci numbers.
long fib(const long n) {
	if (n < 2)
		return n;
	else
		return fib(n - 1) + fib(n - 2);
}


// Define a class for the server that inherits from the ROS Node class, as usual.
class BasicActionServer : public rclcpp::Node {
private:
	// Simplify some of the types
	using Fibonacci = rob599_msgs::action::Fibonacci;
	using FibonacciGoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;

	// Next, we're going to define the callbacks.  We're going to do this before we define the constructor
	// because we're using the auto keyword for the callback return types.  We need to let the compiler
	// process these first so that it can figure out the function signatures before using them in a
	// std::bind call.  If we put the constructor first, like we usually do, the compile fails.

	// The handle goal callback.  This will tell the client if we're going to accept the request.  We're
	// going to accept all requests in this example.
	auto handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Fibonacci::Goal> goal) {
		// Avoid warnings about unused parameters.
		(void)uuid;
		(void)goal;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	};

	// The cancel callback.  This is called when a cancel request comes in.  We return a code to the
	// client indicating if we're going to cancel the goal.
	auto handle_cancel(const std::shared_ptr<FibonacciGoalHandle> goal_handle) {
		RCLCPP_INFO(this->get_logger(), "Recieved goal cancel request");		

		(void)goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	};

	// This callback is called when the goal is accepted (by the handle_goal callback, above).  In this
	// version, we're going to spin up a thread to handle the work. This means that we will be able to 
	// handle multiple simultaneous action requests, each of which will run in a separate thread.  If
	// the action is something that shouldn't run in parallel, you can handle it here.
	auto handle_accepted(const std::shared_ptr<FibonacciGoalHandle> goal_handle) {
		// Define a lambda to start the thread, since this will take care of binding the arguments),
		// and spin up a thread and detach it.
		auto execute_in_thread = [this, goal_handle](){return this->execute_action(goal_handle);};
		std::thread{execute_in_thread}.detach();
	};

public:
	// Constructor.
	BasicActionServer() :Node("fibber") {
		// Allocate an action server.  We need a reference to this instance, a name, and the three
		// callbacks tha the server will use.
		server_ = rclcpp_action::create_server<Fibonacci>(this, "fibonacci",
			std::bind(&BasicActionServer::handle_goal, this, _1, _2),
			std::bind(&BasicActionServer::handle_cancel, this, _1),
			std::bind(&BasicActionServer::handle_accepted, this, _1)
			);
	}

private:
	// A variable for the server instance.
	rclcpp_action::Server<Fibonacci>::SharedPtr server_;

	// This is the function that implements the action.  It's going to run in a detached thread.
	void execute_action(const std::shared_ptr<FibonacciGoalHandle> goal_handle) {
		// We're going to artificially slow down the action, so that we can see the feedback come
		// into the client.  2 Hz should do it.
		rclcpp::Rate rate(2);

		// Extract the goal from the goal handle.
		auto goal = goal_handle->get_goal();

		RCLCPP_INFO(this->get_logger(), "Got %ld", goal->number);		

		// Allocate the feedback and result.
		auto feedback = std::make_shared<Fibonacci::Feedback>();
		auto result = std::make_shared<Fibonacci::Result>();

		// An output string stream to build the partial feedback.  Since the logging functionality
		// uses printf-like formatting, we're going to use string streams to assemble a string,
		// then send it to the logger.
		std::ostringstream os;

		// Do the calculation.
		for(long i = 0; i <= goal->number; ++i) {
			// Check to see if the action is in a canceled state.  If it is, then inform the client
			// and bail.
			if (goal_handle->is_canceling()) {
				// Let the client know.
				goal_handle->canceled(result);

				RCLCPP_INFO(this->get_logger(), "Goal canceled.");		

				// And, we're done.  This will end the thread.
				return;
			}

			// Add the next number to the result sequence.  In C++ the list is a vector.
			result->sequence.push_back(fib(i));

			// Add the just-calculated number to the end of the loggingstring.
			os << result->sequence[i] << ' ';

			RCLCPP_INFO(this->get_logger(), "Partial result: %s", os.str().c_str());

			// Set the feedback message data field, and send the feedback.
			feedback->progress = i;
			goal_handle->publish_feedback(feedback);

			// Wait a little before looping so that we can see all of the feedback in the client.
			rate.sleep();
		}

		if (rclcpp::ok()) {
			goal_handle->succeed(result);
			RCLCPP_INFO(this->get_logger(), "Final result: %s", os.str().c_str());
		}
	}
};


// This is the entry point of the node.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Instantiate the server.
	auto server = std::make_shared<BasicActionServer>();

	// Give control over to ROS.
	rclcpp::spin(server);

	// Make sure everything is shut down cleanly.
	rclcpp::shutdown();

	// And, we're done.
	return 0;
}