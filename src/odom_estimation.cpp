// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

// ros header
#include <tf2/LinearMath/Quaternion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// c++ header
#include <thread>
#include <chrono>

// ros header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "floam/odom_estimation_node.hpp"


OdomEstimationNode::OdomEstimationNode()
  : Node("odom_estimation_node"), laserPath_{}, is_odom_inited_(false), total_time_(0.0), total_frame_(0)
{
  int scan_line = 64;
  double scan_period = 0.1;
  double vertical_angle = 2.0;
  double max_dist = 60.0;
  double min_dist = 3.0;
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

  odomEstimation_.init(map_resolution);

  subEdgeLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "laser_cloud_edge", 100, std::bind(&OdomEstimationNode::velodyneEdgeHandler, this, std::placeholders::_1));
  subSurfLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "laser_cloud_surf", 100, std::bind(&OdomEstimationNode::velodyneSurfHandler, this, std::placeholders::_1));

  pubLaserOdometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
  pubLaserPath_ = this->create_publisher<nav_msgs::msg::Path>("odom_path", 100);

  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void OdomEstimationNode::odom_estimation()
{
  while (1) {
    if (!pointCloudEdgeBuf_.empty() && !pointCloudSurfBuf_.empty()) {
      // read data
      mutex_lock_.lock();
      if (!pointCloudSurfBuf_.empty() && (pointCloudSurfBuf_.front()->header.stamp.sec < pointCloudEdgeBuf_.front()->header.stamp.sec - 0.5*lidar_param_.scan_period)) {
        pointCloudSurfBuf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned with extra point cloud, pls check your data --> odom correction");
        mutex_lock_.unlock();
        continue;
      }

      if (!pointCloudEdgeBuf_.empty() && (pointCloudEdgeBuf_.front()->header.stamp.sec < pointCloudSurfBuf_.front()->header.stamp.sec - 0.5*lidar_param_.scan_period)) {
        pointCloudEdgeBuf_.pop();
        RCLCPP_WARN(this->get_logger(), "time stamp unaligned with extra point cloud, pls check your data --> odom correction");
        mutex_lock_.unlock();
        continue;
      }

      // if time aligned
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*pointCloudEdgeBuf_.front(), *pointcloud_edge_in);
      pcl::fromROSMsg(*pointCloudSurfBuf_.front(), *pointcloud_surf_in);

      rclcpp::Time pointcloud_time = pointCloudSurfBuf_.front()->header.stamp;
      pointCloudEdgeBuf_.pop();
      pointCloudSurfBuf_.pop();
      mutex_lock_.unlock();

      if (is_odom_inited_ == false) {
        odomEstimation_.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
        is_odom_inited_ = true;
        RCLCPP_INFO(this->get_logger(), "odom inited");
      } else {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        odomEstimation_.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
        end = std::chrono::system_clock::now();
        std::chrono::duration<float> elapsed_seconds = end - start;
        total_frame_++;
        float time_temp = elapsed_seconds.count() * 1000;
        total_time_ += time_temp;
        RCLCPP_INFO(this->get_logger(), "average odom estimation time %f ms\n", total_time_/total_frame_);
      }

      Eigen::Quaterniond q_current(odomEstimation_.odom.rotation());
      //q_current.normalize();
      Eigen::Vector3d t_current = odomEstimation_.odom.translation();

      geometry_msgs::msg::TransformStamped t;
      //t.header.stamp = this->get_clock()->now();
      t.header.stamp = pointcloud_time;
      t.header.frame_id = "map";
      t.child_frame_id = "base_link";

      t.transform.translation.x = t_current.x();
      t.transform.translation.y = t_current.y();
      t.transform.translation.z = t_current.z();

      tf2::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      br_->sendTransform(t);

      // publish odometry
      nav_msgs::msg::Odometry laserOdometry;
      laserOdometry.header.frame_id = "map";
      laserOdometry.child_frame_id = "base_link";
      laserOdometry.header.stamp = pointcloud_time;
      laserOdometry.pose.pose.orientation.x = q_current.x();
      laserOdometry.pose.pose.orientation.y = q_current.y();
      laserOdometry.pose.pose.orientation.z = q_current.z();
      laserOdometry.pose.pose.orientation.w = q_current.w();
      laserOdometry.pose.pose.position.x = t_current.x();
      laserOdometry.pose.pose.position.y = t_current.y();
      laserOdometry.pose.pose.position.z = t_current.z();
      pubLaserOdometry_->publish(laserOdometry);

      // publish path
      geometry_msgs::msg::PoseStamped laserPose;
      laserPose.header = laserOdometry.header;
      laserPose.pose = laserOdometry.pose.pose;

      laserPath_.header = laserOdometry.header;
      laserPath_.poses.push_back(laserPose);
      pubLaserPath_->publish(laserPath_);
    }

    //sleep 2 ms every time
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

void OdomEstimationNode::velodyneSurfHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
{
  mutex_lock_.lock();
  pointCloudSurfBuf_.push(laserCloudMsg);
  mutex_lock_.unlock();
}

void OdomEstimationNode::velodyneEdgeHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
{
  mutex_lock_.lock();
  pointCloudEdgeBuf_.push(laserCloudMsg);
  mutex_lock_.unlock();
}
