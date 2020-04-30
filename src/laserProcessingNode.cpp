
// Author of ALOAM_Optimized: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubLaserCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
   
}

void laser_processing(){
    while(1){
        if(!pointCloudBuf.empty()){
            //read data
            mutex_lock.lock();
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
            ros::Time pointcloud_time = (pointCloudBuf.front())->header.stamp;
            pointCloudBuf.pop();
            mutex_lock.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_less_sharp(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_sharp(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_less_flat(new pcl::PointCloud<pcl::PointXYZI>());            
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_flat(new pcl::PointCloud<pcl::PointXYZI>());

            laserProcessing.preFiltering(pointcloud_in, pointcloud_filtered);
            laserProcessing.featureExtraction(pointcloud_filtered,pointcloud_sharp,pointcloud_less_sharp,pointcloud_flat,pointcloud_less_flat);

            sensor_msgs::PointCloud2 laserCloudFilteredMsg;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "/velodyne";
            pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

            sensor_msgs::PointCloud2 cornerPointsSharpMsg;
            pcl::toROSMsg(*pointcloud_sharp, cornerPointsSharpMsg);
            cornerPointsSharpMsg.header.stamp = pointcloud_time;
            cornerPointsSharpMsg.header.frame_id = "/velodyne";
            pubCornerPointsSharp.publish(cornerPointsSharpMsg);

            sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
            pcl::toROSMsg(*pointcloud_less_sharp, cornerPointsLessSharpMsg);
            cornerPointsLessSharpMsg.header.stamp = pointcloud_time;
            cornerPointsLessSharpMsg.header.frame_id = "/velodyne";
            pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

            sensor_msgs::PointCloud2 surfPointsFlatMsg;
            pcl::toROSMsg(*pointcloud_flat, surfPointsFlatMsg);
            surfPointsFlatMsg.header.stamp = pointcloud_time;
            surfPointsFlatMsg.header.frame_id = "/velodyne";
            pubSurfPointsFlat.publish(surfPointsFlatMsg);

            sensor_msgs::PointCloud2 surfPointsLessFlatMsg;
            pcl::toROSMsg(*pointcloud_less_flat, surfPointsLessFlatMsg);
            surfPointsLessFlatMsg.header.stamp = pointcloud_time;
            surfPointsLessFlatMsg.header.frame_id = "/velodyne";
            pubSurfPointsLessFlat.publish(surfPointsLessFlatMsg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    laserProcessing.init(lidar_param);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 100, velodyneHandler);

    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 100);

    pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100);

    pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 100); 

    pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 100); 

    pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100); 

    std::thread laser_processing_process{laser_processing};

    ros::spin();

    return 0;
}
