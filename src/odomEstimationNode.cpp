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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.hpp"
#include "odomEstimationClass.hpp"

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudSurfBuf;
lidar::Lidar lidar_param;
std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometry;
void velodyneSurfHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

bool is_odom_inited = false;
double total_time =0;
int total_frame=0;
void odom_estimation(){
    while(1){
        if(!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.sec<pointCloudEdgeBuf.front()->header.stamp.sec-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                RCLCPP_WARN_ONCE(rclcpp::get_logger("rclcpp"),"time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.sec<pointCloudSurfBuf.front()->header.stamp.sec-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                RCLCPP_WARN_ONCE(rclcpp::get_logger("rclcpp"),"time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;  
            }
            //if time aligned 

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            rclcpp::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if(is_odom_inited == false){
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"odom inited");
            }else{
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time+=time_temp;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"average odom estimation time %f ms \n \n", total_time/total_frame);
            }



            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            //q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            // static tf2_ros::TransformBroadcaster br;
            // tf2::Transform transform;
            // transform.setOrigin( tf2::Vector3(t_current.x(), t_current.y(), t_current.z()) );
            // tf2::Quaternion q(q_current.x(),q_current.y(),q_current.z(),q_current.w());
            // transform.setRotation(q);
            // br.sendTransform(tf2::StampedTransform(transform, rclcpp::Clock().now(), "map", "base_link"));

            if(broadcaster){
                geometry_msgs::msg::TransformStamped transform;
                transform.header.stamp = rclcpp::Clock().now();
                transform.header.frame_id = "map";
                transform.child_frame_id = "base_link";
                transform.transform.translation.x = t_current.x();
                transform.transform.translation.y = t_current.y();
                transform.transform.translation.z = t_current.z();

            }    

            // publish odometry
            nav_msgs::msg::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry->publish(laserOdometry);

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("main");

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period= 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    nh->get_parameter("/scan_period", scan_period); 
    nh->get_parameter("/vertical_angle", vertical_angle); 
    nh->get_parameter("/max_dis", max_dis);
    nh->get_parameter("/min_dis", min_dis);
    nh->get_parameter("/scan_line", scan_line);
    nh->get_parameter("/map_resolution", map_resolution);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);

    odomEstimation.init(lidar_param, map_resolution);
    auto subEdgeLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);

    auto subSurfLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);

    
    pubLaserOdometry = nh->create_publisher<nav_msgs::msg::Odometry>("/odom", 100); 

    std::thread odom_estimation_process{odom_estimation};

    rclcpp::spin(nh);

    return 0;
}

