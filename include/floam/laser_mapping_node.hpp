// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

// c++ header
#include <mutex>
#include <queue>

// local header
#include "floam/lidar.hpp"
#include "floam/laser_mapping_class.hpp"


class LaserMappingNode: public rclcpp::Node
{
  public:
    LaserMappingNode();
    void laser_mapping();

  private:
    LaserMappingClass laserMapping_;
    std::queue<nav_msgs::msg::Odometry::ConstSharedPtr> odometryBuf_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointCloudBuf_;
    std::mutex mutex_lock_;
    lidar::Lidar lidar_param_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubMap_;

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr odomMsg);
    void velodyneHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg);
};
