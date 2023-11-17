// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

#pragma once

// ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// c++ lib
#include <mutex>
#include <queue>

// local lib
#include "floam/lidar.hpp"
#include "floam/laserProcessingClass.hpp"


class LaserProcessingNode: public rclcpp::Node
{
  public:
    LaserProcessingNode();
    void laser_processing();

  private:
    LaserProcessingClass laserProcessing_;
    std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointCloudBuf_;
    std::mutex mutex_lock_;
    lidar::Lidar lidar_param_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgePoints_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPoints_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFiltered_;

    void lidarHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg);

    double total_time_;
    int total_frame_;
};
