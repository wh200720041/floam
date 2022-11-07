// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

// ros header
#include <pcl_conversions/pcl_conversions.h>

// c++ header
#include <thread>
#include <chrono>

// local header
#include "floam/laser_processing_node.hpp"


LaserProcessingNode::LaserProcessingNode()
  : Node("laser_processing_node"), total_time_(0.0), total_frame_(0)
{
  // set the default value of the parameters
  int scan_line = 64;
  double scan_period = 0.1;
  double vertical_angle = 2.0;
  double max_dist = 60.0;
  double min_dist = 2.0;
  this->declare_parameter("scan_line", scan_line);
  this->declare_parameter("scan_period", scan_period);
  this->declare_parameter("vertical_angle", vertical_angle);
  this->declare_parameter("max_dist", max_dist);
  this->declare_parameter("min_dist", min_dist);

  // load from parameter if provided
  scan_line = this->get_parameter("scan_line").get_parameter_value().get<int>();
  scan_period = this->get_parameter("scan_period").get_parameter_value().get<double>();
  vertical_angle = this->get_parameter("vertical_angle").get_parameter_value().get<double>();
  max_dist = this->get_parameter("max_dist").get_parameter_value().get<double>();
  min_dist = this->get_parameter("min_dist").get_parameter_value().get<double>();

  lidar_param_.setScanPeriod(scan_period);
  lidar_param_.setVerticalAngle(vertical_angle);
  lidar_param_.setLines(scan_line);
  lidar_param_.setMaxDistance(max_dist);
  lidar_param_.setMinDistance(min_dist);

  laserProcessing_.init(lidar_param_);

  subLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points", 100, std::bind(&LaserProcessingNode::velodyneHandler, this, std::placeholders::_1));

  pubLaserCloudFiltered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points_filtered", 100);
  pubEdgePoints_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_edge", 100);
  pubSurfPoints_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_surf", 100);
}

void LaserProcessingNode::laser_processing()
{
  while (1) {
    if (!pointCloudBuf_.empty()) {
      //read data
      mutex_lock_.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*pointCloudBuf_.front(), *pointcloud_in);
      rclcpp::Time pointcloud_time = (pointCloudBuf_.front())->header.stamp;
      pointCloudBuf_.pop();
      mutex_lock_.unlock();

      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

      std::chrono::time_point<std::chrono::system_clock> start, end;
      start = std::chrono::system_clock::now();
      laserProcessing_.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf);
      end = std::chrono::system_clock::now();
      std::chrono::duration<float> elapsed_seconds = end - start;
      total_frame_++;
      float time_temp = elapsed_seconds.count() * 1000;
      total_time_ += time_temp;
      //RCLCPP_INFO(this->get_logger(), "Average laser processing time %f ms \n \n", total_time_/total_frame_);

      sensor_msgs::msg::PointCloud2 laserCloudFilteredMsg;
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
      *pointcloud_filtered += *pointcloud_edge;
      *pointcloud_filtered += *pointcloud_surf;
      pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
      laserCloudFilteredMsg.header.stamp = pointcloud_time;
      laserCloudFilteredMsg.header.frame_id = "base_link";
      pubLaserCloudFiltered_->publish(laserCloudFilteredMsg);

      sensor_msgs::msg::PointCloud2 edgePointsMsg;
      pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
      edgePointsMsg.header.stamp = pointcloud_time;
      edgePointsMsg.header.frame_id = "base_link";
      pubEdgePoints_->publish(edgePointsMsg);

      sensor_msgs::msg::PointCloud2 surfPointsMsg;
      pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
      surfPointsMsg.header.stamp = pointcloud_time;
      surfPointsMsg.header.frame_id = "base_link";
      pubSurfPoints_->publish(surfPointsMsg);
    }

    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void LaserProcessingNode::velodyneHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
{
  mutex_lock_.lock();
  pointCloudBuf_.push(laserCloudMsg);
  mutex_lock_.unlock();
}
