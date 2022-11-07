// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

#pragma once

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "lidar.hpp"


// points covariance class
class Double2d
{
  public:
    int id;
    double value;
    Double2d(int id_in, double value_in);
};

// points info class
class PointsInfo
{
  public:
    int layer;
    double time;
    PointsInfo(int layer_in, double time_in);
};

class LaserProcessingClass
{
  public:
    LaserProcessingClass() = default;
    void init(lidar::Lidar lidar_param_in);
    void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
      pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
      pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);
    void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
      std::vector<Double2d>& cloudCurvature,
      pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
      pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);

  private:
    lidar::Lidar lidar_param;
};
