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
#include "lidar.hpp"
#include "laserProcessingClass.hpp"




#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(floam_preprocessing, handle) {
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
    lidar::Lidar lidar_param;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    
    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);


    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubEdgePoints;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSurfPoints;;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFiltered;

    py::class_< LaserProcessingClass >(handle, "LaserProcessingClass")
        .def(py::init<>())
        .def("init_python" , &LaserProcessingClass::init_python);
        // .def("featureExtraction" , &LaserProcessingClass::featureExtraction)
        // .def("featureExtractionFromSector" , &LaserProcessingClass::featureExtractionFromSector);
        
    
#ifdef VERSION_INFO
    handle.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    handle.attr("__version__") = "dev";
#endif
}
