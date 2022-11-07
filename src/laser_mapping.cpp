// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

// ros header
#include <pcl_conversions/pcl_conversions.h>

// c++ header
#include <thread>
#include <chrono>

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "floam/laser_mapping_node.hpp"


LaserMappingNode::LaserMappingNode()
  : Node("laser_mapping_node")
{
  int scan_line = 64;
  double scan_period = 0.1;
  double vertical_angle = 2.0;
  double max_dist = 60.0;
  double min_dist = 2.0;
  double map_resolution = 0.4;

  this->declare_parameter("scan_line", scan_line);
  this->declare_parameter("scan_period", scan_period);
  this->declare_parameter("vertical_angle", vertical_angle);
  this->declare_parameter("max_dist", max_dist);
  this->declare_parameter("min_dist", min_dist);
  this->declare_parameter("map_resolution", map_resolution);

  // load from parameter if provided
  scan_line = this->get_parameter("scan_line").get_parameter_value().get<int>();
  scan_period = this->get_parameter("scan_period").get_parameter_value().get<double>();
  vertical_angle = this->get_parameter("vertical_angle").get_parameter_value().get<double>();
  max_dist = this->get_parameter("max_dist").get_parameter_value().get<double>();
  min_dist = this->get_parameter("min_dist").get_parameter_value().get<double>();
  map_resolution = this->get_parameter("map_resolution").get_parameter_value().get<double>();

  lidar_param_.setScanPeriod(scan_period);
  lidar_param_.setVerticalAngle(vertical_angle);
  lidar_param_.setLines(scan_line);
  lidar_param_.setMaxDistance(max_dist);
  lidar_param_.setMinDistance(min_dist);

  laserMapping_.init(map_resolution);

  subLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points_filtered", 100, std::bind(&LaserMappingNode::velodyneHandler, this, std::placeholders::_1));
  subOdometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 100, std::bind(&LaserMappingNode::odomCallback, this, std::placeholders::_1));

  pubMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 100);
}

void LaserMappingNode::laser_mapping()
{
  while (1) {
    if (!odometryBuf_.empty() && !pointCloudBuf_.empty()) {
      // read data
      mutex_lock_.lock();
      if (!pointCloudBuf_.empty() && pointCloudBuf_.front()->header.stamp.sec < odometryBuf_.front()->header.stamp.sec - 0.5*lidar_param_.scan_period) {
        pointCloudBuf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned error and pointcloud discarded, pls check your data --> laser mapping node");
        mutex_lock_.unlock();
        continue;
      }

      if (!odometryBuf_.empty() && odometryBuf_.front()->header.stamp.sec < pointCloudBuf_.front()->header.stamp.sec - 0.5*lidar_param_.scan_period) {
        odometryBuf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned with path final, pls check your data --> laser mapping node");
        mutex_lock_.unlock();
        continue;
      }

      // if time aligned
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*pointCloudBuf_.front(), *pointcloud_in);
      rclcpp::Time pointcloud_time = (pointCloudBuf_.front())->header.stamp;

      Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
      current_pose.rotate(Eigen::Quaterniond(odometryBuf_.front()->pose.pose.orientation.w, odometryBuf_.front()->pose.pose.orientation.x, odometryBuf_.front()->pose.pose.orientation.y, odometryBuf_.front()->pose.pose.orientation.z));
      current_pose.pretranslate(Eigen::Vector3d(odometryBuf_.front()->pose.pose.position.x, odometryBuf_.front()->pose.pose.position.y, odometryBuf_.front()->pose.pose.position.z));
      pointCloudBuf_.pop();
      odometryBuf_.pop();
      mutex_lock_.unlock();

      laserMapping_.updateCurrentPointsToMap(pointcloud_in, current_pose);

      pcl::PointCloud<pcl::PointXYZI>::Ptr pc_map = laserMapping_.getMap();
      sensor_msgs::msg::PointCloud2 PointsMsg;
      pcl::toROSMsg(*pc_map, PointsMsg);
      PointsMsg.header.stamp = pointcloud_time;
      PointsMsg.header.frame_id = "map";
      pubMap_->publish(PointsMsg);
    }

    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void LaserMappingNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg)
{
  mutex_lock_.lock();
  odometryBuf_.push(odomMsg);
  mutex_lock_.unlock();
}

void LaserMappingNode::velodyneHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
{
  mutex_lock_.lock();
  pointCloudBuf_.push(laserCloudMsg);
  mutex_lock_.unlock();
}
