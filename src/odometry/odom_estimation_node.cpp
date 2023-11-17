// Author of FLOAM: Wang Han
// ROS2 Migration: Yi-Chen Zhang

// ros header
#include <rclcpp/rclcpp.hpp>

// c++ header
#include <thread>

// local header
#include "floam/odom_estimation_node.hpp"


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto oen_ptr {std::make_shared<OdomEstimationNode>()};
  std::thread odom_estimation_process(&OdomEstimationNode::odom_estimation, oen_ptr);
  rclcpp::spin(oen_ptr);
  rclcpp::shutdown();

  return 0;
}
