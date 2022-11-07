// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

#pragma once

namespace lidar {

class Lidar
{
  public:
    Lidar() = default;

    void setScanPeriod(double scan_period_in);
    void setLines(int num_lines_in);
    void setVerticalAngle(double vertical_angle_in);
    void setVerticalResolution(double vertical_angle_resolution_in);
    void setMaxDistance(double max_distance_in);
    void setMinDistance(double min_distance_in);

    double max_distance;
    double min_distance;
    int num_lines;
    double scan_period;
    int points_per_line;
    double horizontal_angle_resolution;
    double horizontal_angle;
    double vertical_angle_resolution;
    double vertical_angle;
};

}
