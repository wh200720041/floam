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
#include "floam/laserProcessingClass.hpp"




#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)


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

    
    
    py::class_< LaserProcessingClass >(handle, "LaserProcessingClass")
        .def(py::init<>())
        .def("init_python" , &LaserProcessingClass::init_python)
        .def("featureExtraction_python" , &LaserProcessingClass::featureExtraction_python)
        .def("get_edge" , &LaserProcessingClass::get_edge)
        .def("get_surf" , &LaserProcessingClass::get_surf);
        // .def("featureExtractionFromSector" , &LaserProcessingClass::featureExtractionFromSector);
        
    
// #ifdef VERSION_INFO
//     handle.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
// #else
//     handle.attr("__version__") = "dev";
// #endif
    // return handle.ptr();
}
