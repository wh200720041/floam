cmake_minimum_required(VERSION 3.5)
project(floam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

add_executable(laser_processing_node
  src/laser_processing_node.cpp
  src/laser_processing.cpp
  src/laser_processing_class.cpp
  src/lidar.cpp
)

ament_target_dependencies(laser_processing_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(laser_processing_node
  ${PCL_LIBRARIES}
)

add_executable(odom_estimation_node
  src/odom_estimation_node.cpp
  src/odom_estimation.cpp
  src/odom_estimation_class.cpp
  src/lidar_optimization.cpp
  src/lidar.cpp
)

ament_target_dependencies(odom_estimation_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_conversions
  Eigen3
  #CERES
)

target_link_libraries(odom_estimation_node
  ${PCL_LIBRARIES}
  #${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(laser_mapping_node
  src/laser_mapping_node.cpp
  src/laser_mapping.cpp
  src/laser_mapping_class.cpp
  src/lidar.cpp
)

ament_target_dependencies(laser_mapping_node
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)

target_link_libraries(laser_mapping_node
  ${PCL_LIBRARIES}
)

install(TARGETS laser_processing_node odom_estimation_node laser_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
