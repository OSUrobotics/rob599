#include <rclcpp/rclcpp.hpp>

// Include the definition of the Doubler message type.  Note that the filename is doubler.hpp.
// All CamelCased names are translated to camel_cased.hpp filenames for included message types.
#include <rob599_msgs/srv/doubler.hpp>

// We need this in order to set up the service.  This is the C++ way of letting us specify
// callback parameters.  For services, we need two placeholders, _1 and _2.
using std::placeholders::_1;
using std::placeholders::_2;


// Service node that inherits from Node, as usual.
class BasicService : public rclcpp::Node {
public:
	// Initialize the node, 
	BasicService() :Node("service") {
		// Create the service.  The type goes in the template parameter, as usual, and we need
		// two parameters.
		service_ = this->create_service<rob599_msgs::srv::Doubler>("doubler", std::bind(&BasicService::service_callback, this, _1, _2));
	}

	// A callback to handle the service request.  The has parameters for the request and
	// the response.
	void service_callback(
		const std::shared_ptr<rob599_msgs::srv::Doubler::Request> request,
		const std::shared_ptr<rob599_msgs::srv::Doubler::Response> response) {

		// Calculate the response and fill in the response field.  The idiom in C++ is to not
		// return anything from the callback explicitly, and rely on setting the fields in the
		// response variable.
		response->doubled = request->number * 2;

		RCLCPP_INFO(this->get_logger(), "Got service request for %li", request->number);
	}

private:
	// A variable to hold the pointer to the service.  Note the templated type for the Doubler
	// message type.
	rclcpp::Service<rob599_msgs::srv::Doubler>::SharedPtr service_;
};


int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node.
	auto service = std::make_shared<BasicService>();

	// Give control over to the ROS event handler.
	rclcpp::spin(service);

	// Once the event handler is done, make sure we're completely shut down.
	rclcpp::shutdown();

	return 0;
}