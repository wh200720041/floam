// Author of FLOAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

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