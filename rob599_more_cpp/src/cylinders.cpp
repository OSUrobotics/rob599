// cylinders.cpp
//
// Bill Smart
//
// This is an example of using the Point Cloud Library in ROS 2
// to find cylinders in point cloud data.


// Include the basic ROS 2 stuff.
#include <rclcpp/rclcpp.hpp>

// We're going to be dealing with point clouds.
#include <sensor_msgs/msg/point_cloud2.hpp>

// Include the point cloud library (PCL) stuff.
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>


// Binding parameters for the callback.
using std::placeholders::_1;


// The idiom is always to create a Node subclass.
class CylinderFinder : public rclcpp::Node {
public:
	// Simplify the syntax a bit.
	using PointCloud2 = sensor_msgs::msg::PointCloud2;

	CylinderFinder() :Node("cylinder_finder") {
		// Create a publisher for the modified point cloud.
		publisher_ = this->create_publisher<PointCloud2>("modified", 1);

		// Create a subscriber for the point cloud.  We're only going to keep an input
		// buffer of size 1, since this might be a slow process.
		subscriber_ = this->create_subscription<PointCloud2>("cloud", 1, std::bind(&CylinderFinder::callback, this, _1));
	}

private:
	rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
	rclcpp::Subscription<PointCloud2>::SharedPtr subscriber_;

	void callback(const PointCloud2::SharedPtr msg) {
		RCLCPP_INFO(this->get_logger(), "Got a point cloud!!!");

		// Translate the ROS PointCloud2 to a PCL PointCloud.
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(*msg, *pcl_cloud);

		// Make another PCL PointCloud for the result.
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

		// Make a kd-tree to use in the normal calculation.  For cylinder-finding, we need
		// to use normals.  We're going to make a Kd-tree for efficient searching, the use
		// this to calculate the normals.  If we're looking for a simpler shape, like a plane,
		// we might not have to do this.
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

		// Set the parameters.  The data are relatively clean, so we can use a small K for the
		// Normal calculation.
		ne.setSearchMethod(tree);
		ne.setInputCloud(pcl_cloud);
		ne.setKSearch(10);
		ne.compute(*cloud_normals);

		// Allocate a segmentation model, a collection of point indicies, and some model
		// coefficients.  We use the segmentation model to extract what we want, the store
		// the points that "belong" to the model in the inliers variable.  The parameters
		// for the model we fit are stored in the coefficients varaible.
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);		

		// Set the model type, algorithm, distance threshold, and the point cloud we're
		// using as input.  Then, run the algorith, to produce the inliers (points that
		// "belong" to the fit model) and model coefficients.
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.05);
		seg.setRadiusLimits(0, 1);
		seg.setInputCloud(pcl_cloud);
		seg.setInputNormals(cloud_normals);
		seg.segment(*inliers, *coefficients);

		// Log how many points are claimed by the model.
		RCLCPP_INFO(this->get_logger(), "Got %li inliers.", inliers->indices.size());

		// Filter the model points out of the cloud.  Set the input cloud, the indexes to
		// extract, whether we want to extract those indices or all the other indices.  We
		// want to keep the inliers, so the argument is false.
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(pcl_cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*pcl_cloud_filtered);

		// Convert PCL PointCloud to a ROS PointCloud2.
		PointCloud2 ros_cloud;
		pcl::toROSMsg(*pcl_cloud_filtered, ros_cloud);

		// Set the header information.
		ros_cloud.header = msg->header;

		// And, publish it out.
		this->publisher_->publish(ros_cloud);
	}
};


// This is the entry point for the executable.
int main(int argc, char **argv) {
	// Initialize ROS.
	rclcpp::init(argc, argv);

	// Create a node and store a shared pointer to it.  We're
	// going to use the auto keyword here to make sure we get
	// the type right.
	auto node = std::make_shared<CylinderFinder>();

	// Give control to ROS via the shared pointer.
	rclcpp::spin(node);

	// Once the event handler is done, shut things down nicely.
	rclcpp::shutdown();

	// Main always returns 0 on success.
	return 0;
}
