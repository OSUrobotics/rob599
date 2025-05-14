// cloud_generator.cpp
//
// Bill Smart
//
// This code generates fake point clouds of a specific size, to illustrate the latency behaviour of the
// ROS 2 middleware.


#include <rob599_more_cpp/cloud_generator_node.hpp>


// String literals
#include <chrono>
using namespace std::chrono_literals;

// We're going to set up a callback with a single argument.
using std::placeholders::_1;


CloudGeneratorNode::CloudGeneratorNode() :Node("cloud_generator"), cloud_size_(640 * 480) {
	// A publisher to publish point clouds.  Set this up first.
	publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clouds", 10);

	// A timer to periodically publish point clouds.
	timer_ = this->create_wall_timer(250ms, std::bind(&CloudGeneratorNode::timer_callback, this));

	// We're going to use a parameter to set the size of the point cloud data block. First, we need to
	// declare our parameter.  Make the default the size of a small image.
	this->declare_parameter("cloud_size", 640 * 480);

	// Set up a handler to handle parameter events.  Then register a callback to call when the cloud_size
	// parameter changes.  We assign these to variables, to make sure they don't go out of scope unexpectedly.
	parameter_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
	parameter_callback_handle_ = parameter_subscriber_->add_parameter_callback("cloud_size", std::bind(&CloudGeneratorNode::parameter_callback, this, _1));
}


CloudGeneratorNode::~CloudGeneratorNode() {
}


// The parameter event callback returns a void, like all callbacks, and takes a single argument of type
// rclcpp::Parameter.  This is a composite type, which acts like a union.
void CloudGeneratorNode::parameter_callback(const rclcpp::Parameter &p) {
	RCLCPP_INFO(this->get_logger(), "Got a parameter change: %s to %li", p.get_name().c_str(), p.as_int());

	// Put a mutex around this to make sure we're thread-safe.  We have to extract the value by explicitly
	// calling as_int().
	parameter_mutex_.lock();
	cloud_size_ = p.as_int();
	parameter_mutex_.unlock();
}


// The timer callback will periodically publish a point cloud.  We use a PointCloud2 in this case because
// it has a variable-sized payload, which will be useful for our latency experiments.  We're not actually
// going to construct a meaningful point cloud in this code.
void CloudGeneratorNode::timer_callback() {
	auto message = sensor_msgs::msg::PointCloud2();

	// Set some of the fields in the PointCloud2 appropriately.
	message.height = 1;
	message.point_step = 1;
	message.is_bigendian = false;
	message.is_dense = true;

	// Put a mutex around this bit in case we get a poorly-timed parameter change from another thread.
	// Add a data payload of the right size in to the PointCloud2.
	parameter_mutex_.lock();
	message.width = cloud_size_;
	message.row_step = cloud_size_;
	message.data = std::vector<unsigned char>(cloud_size_);
	parameter_mutex_.unlock();

	// Set the current time.
	message.header.stamp = this->get_clock()->now();

	publisher_->publish(message);

	RCLCPP_INFO(this->get_logger(), "Published point cloud");
}
