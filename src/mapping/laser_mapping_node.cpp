// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <thread>

// local header
#include "floam/laser_mapping_node.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto lmo_ptr {std::make_shared<LaserMappingNode>()};
  std::thread laser_mapping_process(&LaserMappingNode::laser_mapping, lmo_ptr);
  rclcpp::spin(lmo_ptr);
  rclcpp::shutdown();

  return 0;
}
