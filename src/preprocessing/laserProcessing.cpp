// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

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
#include "floam/laser_processing_node.hpp"


LaserProcessingNode::LaserProcessingNode()
  : Node("laser_processing_node"), total_time_(0.0), total_frame_(0)
{
  // set the default value of the parameters
  int scan_line = 64;
  double scan_period = 0.1;
  double vertical_angle = 2.0;
  double max_dist = 60.0;
  double min_dist = 2.0;
  this->declare_parameter("scan_line", scan_line);
  this->declare_parameter("scan_period", scan_period);
  this->declare_parameter("vertical_angle", vertical_angle);
  this->declare_parameter("max_dist", max_dist);
  this->declare_parameter("min_dist", min_dist);

  // load from parameter if provided
  scan_line = this->get_parameter("scan_line").get_parameter_value().get<int>();
  scan_period = this->get_parameter("scan_period").get_parameter_value().get<double>();
  vertical_angle = this->get_parameter("vertical_angle").get_parameter_value().get<double>();
  max_dist = this->get_parameter("max_dist").get_parameter_value().get<double>();
  min_dist = this->get_parameter("min_dist").get_parameter_value().get<double>();

  lidar_param_.setScanPeriod(scan_period);
  lidar_param_.setVerticalAngle(vertical_angle);
  lidar_param_.setLines(scan_line);
  lidar_param_.setMaxDistance(max_dist);
  lidar_param_.setMinDistance(min_dist);

  laserProcessing_.init(lidar_param_);

  subLaserCloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/pandar", 100, std::bind(&LaserProcessingNode::lidarHandler, this, std::placeholders::_1));

  pubLaserCloudFiltered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_points_filtered", 100);
  pubEdgePoints_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_edge", 100);
  pubSurfPoints_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("laser_cloud_surf", 100);
}


void LaserProcessingNode::lidarHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudMsg)
{
  mutex_lock_.lock();
  pointCloudBuf_.push(laserCloudMsg);
  mutex_lock_.unlock();
}


double total_time =0;
int total_frame=0;

void LaserProcessingNode::laser_processing(){
    while(rclcpp::ok()){
        if(!pointCloudBuf_.empty()){
            //read data
            std::lock_guard<std::mutex> lock(mutex_lock_);
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudBuf_.front(), *pointcloud_in);
            rclcpp::Time pointcloud_time = (pointCloudBuf_.front())->header.stamp;
            pointCloudBuf_.pop();
            mutex_lock_.unlock();

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());

            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            laserProcessing_.featureExtraction(pointcloud_in,pointcloud_edge,pointcloud_surf);
            end = std::chrono::system_clock::now();
            std::chrono::duration<float> elapsed_seconds = end - start;
            total_frame++;
            float time_temp = elapsed_seconds.count() * 1000;
            total_time+=time_temp;
            //ROS_INFO("average laser processing time %f ms \n \n", total_time/total_frame);

            sensor_msgs::msg::PointCloud2 laserCloudFilteredMsg;
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  
            *pointcloud_filtered+=*pointcloud_edge;
            *pointcloud_filtered+=*pointcloud_surf;
            pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
            laserCloudFilteredMsg.header.stamp = pointcloud_time;
            laserCloudFilteredMsg.header.frame_id = "base_link";
            pubLaserCloudFiltered_->publish(laserCloudFilteredMsg);

            sensor_msgs::msg::PointCloud2 edgePointsMsg;
            pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
            edgePointsMsg.header.stamp = pointcloud_time;
            edgePointsMsg.header.frame_id = "base_link";
            pubEdgePoints_->publish(edgePointsMsg);


            sensor_msgs::msg::PointCloud2 surfPointsMsg;
            pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
            surfPointsMsg.header.stamp = pointcloud_time;
            surfPointsMsg.header.frame_id = "base_link";
            pubSurfPoints_->publish(surfPointsMsg);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
    
//     auto node_handler = rclcpp::Node::make_shared("main");
    

//     int scan_line = 64;
//     double vertical_angle = 2.0;
//     double scan_period= 0.1;
//     double max_dis = 60.0;
//     double min_dis = 2.0;

//     node_handler->get_parameter("/scan_period", scan_period); 
//     node_handler->get_parameter("/vertical_angle", vertical_angle); 
//     node_handler->get_parameter("/max_dis", max_dis);
//     node_handler->get_parameter("/min_dis", min_dis);
//     node_handler->get_parameter("/scan_line", scan_line);

//     lidar_param.setScanPeriod(scan_period);
//     lidar_param.setVerticalAngle(vertical_angle);
//     lidar_param.setLines(scan_line);
//     lidar_param.setMaxDistance(max_dis);
//     lidar_param.setMinDistance(min_dis);

//     laserProcessing.init(lidar_param);

//     auto subLaserCloud_ = node_handler->create_subscription<sensor_msgs::msg::PointCloud2>("/pandar", 100, velodyneHandler);

//     pubLaserCloudFiltered = node_handler->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_points_filtered", 100);

//     pubEdgePoints = node_handler->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_edge", 100);

//     pubSurfPoints = node_handler->create_publisher<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf", 100); 

//     std::thread laser_processing_process{laser_processing};

//     rclcpp::spin(node_handler);

//     return 0;
// }

