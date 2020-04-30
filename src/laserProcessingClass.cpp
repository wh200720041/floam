// This is an advanced implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014. 
// Modifier of ALOAM: Tong Qin               qintonguav@gmail.com
//                    Shaozu Cao             saozu.cao@connect.ust.hk
// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author of ALOAM_Optimized: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "laserProcessingClass.h"

void LaserProcessingClass::init(lidar::Lidar lidar_param_in){
    
    lidar_param = lidar_param_in;

    //init filter
    double x_min = -lidar_param.min_distance, y_min = -lidar_param.min_distance, z_min = -lidar_param.min_distance;
    double x_max = +lidar_param.min_distance, y_max = +lidar_param.min_distance, z_max = +lidar_param.min_distance;
    closePointFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    closePointFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    closePointFilter.setNegative(true);
    
    x_min = -lidar_param.max_distance;
    y_min = -lidar_param.max_distance;
    z_min = -lidar_param.max_distance;
    x_max = +lidar_param.max_distance;
    y_max = +lidar_param.max_distance;
    z_max = +lidar_param.max_distance;
    farPointFilter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
    farPointFilter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
    farPointFilter.setNegative(false);

    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

}

void LaserProcessingClass::preFiltering(const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out){
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
    closePointFilter.setInputCloud(pc_in);
    closePointFilter.filter(*cloud_temp);
    farPointFilter.setInputCloud(cloud_temp);
    farPointFilter.filter(*pc_out);

}



void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_sharp, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_lessSharp, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_flat, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_lessFlat){
    //这里不考虑校准问题

    int N_SCANS = lidar_param.num_lines;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans;
    for(int i=0;i<N_SCANS;i++){
        laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
    }

    for (int i = 0; i < (int) pc_in->points.size(); i++)
    {
        int scanID=0;
        double angle = atan(pc_in->points[i].z / sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y)) * 180 / M_PI;
        if (N_SCANS == 16)
        {
            scanID = int((angle + 15) / 2 + 0.5);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 32)
        {
            scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
            if (scanID > (N_SCANS - 1) || scanID < 0)
            {
                continue;
            }
        }
        else if (N_SCANS == 64)
        {   
            if (angle >= -8.83)
                scanID = int((2 - angle) * 3.0 + 0.5);
            else
                scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

            // use [0 50]  > 50 remove outlies 
            if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0)
            {
                continue;
            }
        }
        else
        {
            printf("wrong scan number\n");
        }
        laserCloudScans[scanID]->push_back(pc_in->points[i]); 
        
    }

    for(int i = 0; i < N_SCANS; i++){
        for(int j =0;j<(int)laserCloudScans[i]->points.size();j++){
            laserCloudScans[i]->points[j].intensity = i;
        }

    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_temp(new pcl::PointCloud<pcl::PointXYZI>());
    for(int i = 0; i < N_SCANS; i++){
        if(laserCloudScans[i]->points.size()<131){
            continue;
        }
        
        std::vector<Double2d> cloudCurvature; 
        //cloudCurvature.clear();
        int total_points = laserCloudScans[i]->points.size()-10;
        for(int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++){
            double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
            double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
            double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
            Double2d distance(j,diffX * diffX + diffY * diffY + diffZ * diffZ);
            cloudCurvature.push_back(distance);

        }
        //ROS_WARN("6dengfen %d",laserCloudScans[i].size(),i);
        //需要6等分点
        for(int j=0;j<6;j++){
            int sector_length = (int)(total_points/6);
            int sector_start = sector_length *j;
            int sector_end = sector_length *(j+1)-1;
            if (j==5){
                sector_end = total_points - 1; 
            }
            std::vector<Double2d> subCloudCurvature(cloudCurvature.begin()+sector_start,cloudCurvature.begin()+sector_end); 
            
            featureExtractionFromSector(laserCloudScans[i],subCloudCurvature, pc_out_sharp, pc_out_lessSharp, pc_out_flat, surf_temp);
            
        }

    }


    //注意这里的surfPointsLessFlatScan 是所有剩余 的点经过滤波后的结果
    downSizeFilter.setInputCloud(surf_temp);
    downSizeFilter.filter(*pc_out_lessFlat);

}


void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_edge_sharp, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_edge_lessSharp, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_surf_flat, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_surf_lessFlat){

    //std::sort (sort_id.begin(), sort_id.end(), comp);
    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d & a, const Double2d & b)
    { 
        return a.value < b.value; 
    });

    //选出对应的特征点
    int largestPickedNum = 0;
    std::vector<int> picked_points;
    //ROS_WARN("max point:%f, %f",cloudCurvature[20].value,cloudCurvature[cloudCurvature.size()-21].value);
    int point_info_count =0;
    for (int i = cloudCurvature.size()-1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id; 
        if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            if(cloudCurvature[i].value <= 0.1){
                break;
            }
            
            largestPickedNum++;
            picked_points.push_back(ind);
            
            if(largestPickedNum <= 2){
                //find all points
                pc_edge_sharp->push_back(pc_in->points[ind]);
                pc_edge_lessSharp->push_back(pc_in->points[ind]);
                point_info_count++;
            }else if (largestPickedNum <= 20){
                pc_edge_lessSharp->push_back(pc_in->points[ind]);
                point_info_count++;
            }else{
                break;
            }

            for(int k=1;k<=5;k++){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }
            for(int k=-1;k>=-5;k--){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }

        }
    }

    //find flat points
    point_info_count =0;
    int smallestPickedNum = 0;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat_temp(new pcl::PointCloud<pcl::PointXYZI>());
    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    {
        int ind = cloudCurvature[i].id; 

        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            if(cloudCurvature[i].value > 0.1){
                //ROS_WARN("extracted feature not qualified, please check lidar");
                break;
            }
            //if the points are not selected before, then add into library
            smallestPickedNum++;
            picked_points.push_back(ind);
            //ROS_WARN("dfgdfr");

            
            if(smallestPickedNum <= 4){
                //find all points
                pc_surf_flat->push_back(pc_in->points[ind]);
                pc_surf_lessFlat->push_back(pc_in->points[ind]);
                // surfPointsLessFlatInfo.push_back(PointsInfo(scan_num, laserCloudScans_time[scan_num][ind]));
                point_info_count++;
            }
            else{
                break;
            }

            for(int k=1;k<=5;k++){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }
            for(int k=-1;k>=-5;k--){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }

        }
    }
    
    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    {
        int ind = cloudCurvature[i].id; 
        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end())
        {
            pc_surf_lessFlat->push_back(pc_in->points[ind]);
        }
    }
    


}
LaserProcessingClass::LaserProcessingClass(){
    
}

Double2d::Double2d(int id_in, double value_in){
    id = id_in;
    value =value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in){
    layer = layer_in;
    time = time_in;
};
