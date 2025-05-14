// cloud_timer_node.hpp
//
// Bill Smart
//
// Definition of the CloudTimerNode


#ifndef CLOUD_TIMER_NODE_H
#define CLOUD_TIMER_NODE_H


// Include the basic ROS functionality and the message definitions that we need.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>


// The usual idiom, inheriting from Node.
class CloudTimerNode : public rclcpp::Node {
public:
	CloudTimerNode();
	~CloudTimerNode();

private:
	// Subscriber and publisher.
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr average_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_publisher_;
	rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr max_publisher_;

	const size_t buffer_size_ = 10;
	int counter_;
	std::vector<double> buffer_;

	double min_latency_;
	double max_latency_;

	// The callback receives the incoming piont clouds, calculates the latency, and publishes it.
	void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};


#endif
