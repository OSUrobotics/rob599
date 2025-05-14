// cloud_generator_node.cpp
//
// Bill Smart
//
// Definition of the CloudGeneratorNode.


#ifndef CLOUD_GENERATOR_NODE_H
#define CLOUD_GENERATOR_NODE_H


// Include the basic ROS functionality and also the PointCloud2 data type.
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>


// Use the typical idiom, inheriting from Node.
class CloudGeneratorNode : public rclcpp::Node {
public:
	CloudGeneratorNode();
	~CloudGeneratorNode();

private:
	// Publisher and timer.
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Variables for the parameter subscriber and callback handle.
	std::shared_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
	std::shared_ptr<rclcpp::ParameterCallbackHandle> parameter_callback_handle_;

	// A mutex to deal with data from the timer thread.
	std::mutex parameter_mutex_;

	// Store the cloud size in this variable.
	size_t cloud_size_;

	// The parameter event callback returns a void, like all callbacks, and takes a single argument of type
	// rclcpp::Parameter.  This is a composite type, which acts like a union.
	void parameter_callback(const rclcpp::Parameter &p);

	// The timer callback will periodically publish a point cloud.  We use a PointCloud2 in this case because
	// it has a variable-sized payload, which will be useful for our latency experiments.  We're not actually
	// going to construct a meaningful point cloud in this code.
	void timer_callback();
};


#endif
