// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include <pybind11/pybind11.h>

#include "lidar.h"


lidar::Lidar::Lidar(){
 
}


void lidar::Lidar::setLines(double num_lines_in){
    num_lines=num_lines_in;
}


void lidar::Lidar::setVerticalAngle(double vertical_angle_in){
    vertical_angle = vertical_angle_in;
}


void lidar::Lidar::setVerticalResolution(double vertical_angle_resolution_in){
    vertical_angle_resolution = vertical_angle_resolution_in;
}


void lidar::Lidar::setScanPeriod(double scan_period_in){
    scan_period = scan_period_in;
}


void lidar::Lidar::setMaxDistance(double max_distance_in){
	max_distance = max_distance_in;
}

void lidar::Lidar::setMinDistance(double min_distance_in){
	min_distance = min_distance_in;
}


#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(floam_lidar, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: floam_lidar

        .. autosummary::
           :toctree: _generate

           setLines
           setVerticalAngle
           setVerticalResolution
           setScanPeriod
           setMaxDistance
           setMinDistance

    )pbdoc";

    py::class_<lidar::Lidar::Lidar>(m, "lidar")
        .def(py::init<const std::string &, int>())
        .def("setLines", &lidar::Lidar::Lidar::setLines)
    
#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}
