// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include <pybind11/pybind11.h>

#include "floam/lidar.hpp"




#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) {
    return i + j;
}

namespace py = pybind11;

PYBIND11_MODULE(floam_lidar, handle) {
    handle.doc() = R"pbdoc(
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

    // py::class_< lidar::Lidar::Lidar >(handle, "lidar")
    //     .def(py::init<const std::string &, int>())
    //     .def("setLines", &lidar::Lidar::Lidar::setLines);
    
#ifdef VERSION_INFO
    handle.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    handle.attr("__version__") = "dev";
#endif
}
