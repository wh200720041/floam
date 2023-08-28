// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>

// c++ header
#include <mutex>
#include <queue>

// local header
#include "floam/lidar.hpp"
#include "floam/odom_estimation_class.hpp"


class OdomEstimationNode: public rclcpp::Node
{
  public:
    OdomEstimationNode();
    void odom_estimation();

  private:
    OdomEstimationClass odomEstimation_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointCloudEdgeBuf_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointCloudSurfBuf_;
    std::mutex mutex_lock_;
    lidar::Lidar lidar_param_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subEdgeLaserCloud_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subSurfLaserCloud_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubLaserPath_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br_;

    void velodyneSurfHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg);
    void velodyneEdgeHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg);

    nav_msgs::msg::Path laserPath_;

    bool is_odom_inited_;
    double total_time_;
    int total_frame_;
};
