// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <thread>

// local header
#include "floam/laser_processing_node.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto lpn_ptr {std::make_shared<LaserProcessingNode>()};
  std::thread laser_processing_process(&LaserProcessingNode::laser_processing, lpn_ptr);
  rclcpp::spin(lpn_ptr);
  rclcpp::shutdown();

  return 0;
}
