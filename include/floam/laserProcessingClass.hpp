// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LASER_PROCESSING_CLASS_H_
#define _LASER_PROCESSING_CLASS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "lidar.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <iostream>

namespace py = pybind11;

//points covariance class
class Double2d{
public:
	int id;
	double value;
	Double2d(int id_in, double value_in);
};
//points info class
class PointsInfo{
public:
	int layer;
	double time;
	PointsInfo(int layer_in, double time_in);
};


class LaserProcessingClass 
{
    public:
    	LaserProcessingClass();
		void init(lidar::Lidar lidar_param_in);
		void init_python(int scan_line ,double vertical_angle ,double scan_period,double max_dis,double min_dis );
		void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);
		void featureExtraction_python(py::array_t<double>& input1);
		void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);	
		pcl::PointCloud<pcl::PointXYZI> get_edge();
		pcl::PointCloud<pcl::PointXYZI> get_surf();	
	private:
     	lidar::Lidar lidar_param;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out_edge_updated;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out_surf_updated;
	pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in;
};



#endif // _LASER_PROCESSING_CLASS_H_

