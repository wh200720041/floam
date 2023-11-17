#include <pybind11/pybind11.h>

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "floam/lidar.hpp"

#include "floam/odomMappingClass.hpp"





#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(floam_mapping, handle) {
    handle.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: cmake_example

        .. autosummary::
           :toctree: _generate

           add
           subtract
    )pbdoc";

    //LaserProcessingClass laserProcessing;
    // std::mutex mutex_lock;
    // std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudBuf;
    //lidar::Lidar lidar_param;


    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgePoints;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPoints;;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFiltered;

    py::class_< LaserMappingClass  >(handle, "LaserMappingClass ")
        .def(py::init<lidar::Lidar >());
        .def("updateCurrentPointsToMap" , &LaserMappingClass ::featureExtraction)
        .def("getMap" , &LaserMappingClass ::getMap)
        
    
#ifdef VERSION_INFO
    handle.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    handle.attr("__version__") = "dev";
#endif
}
