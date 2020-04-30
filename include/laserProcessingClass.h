#ifndef _LASER_PROCESSING_CLASS_H_
#define _LASER_PROCESSING_CLASS_H_


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include "lidar.h"

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


//这里所有的数据传递全部采用指针或引用的形式来提高传递效率
class LaserProcessingClass 
{
    public:
    	LaserProcessingClass();
		void init(lidar::Lidar lidar_param_in);
		void preFiltering(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out);
		void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_sharp, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_lessSharp, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_flat, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_lessFlat);
		void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_edge_sharp, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_edge_lessSharp, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_surf_flat, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_surf_lessFlat);	
	private:
     	lidar::Lidar lidar_param;

     	pcl::CropBox<pcl::PointXYZI> closePointFilter;
     	pcl::CropBox<pcl::PointXYZI> farPointFilter;
     	pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
};



#endif // _LASER_PROCESSING_CLASS_H_

