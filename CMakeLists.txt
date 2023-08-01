cmake_minimum_required(VERSION 3.5)
project(floam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED  )
find_package(geometry_msgs REQUIRED )
find_package(nav_msgs REQUIRED )
find_package(sensor_msgs REQUIRED )
find_package(rclcpp REQUIRED )
find_package(std_msgs REQUIRED )
find_package(tf2 REQUIRED )
find_package(tf2_ros REQUIRED )
find_package(tf2_geometry_msgs REQUIRED )
find_package(pcl_conversions REQUIRED )
find_package(pcl_ros REQUIRED )
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED )



if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_include_directories(include)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  Ceres
  Eigen3
)



add_executable(floam_laser_processing_node src/laserProcessingNode.cpp src/laserProcessingClass.cpp src/lidar.cpp)
ament_target_dependencies(floam_laser_processing_node  ${dependencies} )
target_link_libraries(floam_laser_processing_node ${CERES_LIBRARIES})
target_include_directories(floam_laser_processing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)     


# add_executable(floam_odom_estimation_node src/odomEstimationNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/odomEstimationClass.cpp)
# #target_link_libraries(floam_odom_estimation_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})
# target_link_libraries(floam_odom_estimation_node ${EIGEN3_LIBRARIES}  ${PCL_LIBRARIES} ${CERES_LIBRARIES})
# #ament_target_dependencies(floam_odom_estimation_node ${PROJECT_NAME} ${dependencies} Eigen3)


# add_executable(floam_laser_mapping_node src/laserMappingNode.cpp src/laserMappingClass.cpp src/lidar.cpp)
# #target_link_libraries(floam_laser_mapping_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})
# target_link_libraries(floam_laser_mapping_node ${EIGEN3_LIBRARIES}  ${PCL_LIBRARIES} ${CERES_LIBRARIES})
# #ament_target_dependencies(floam_laser_mapping_node ${PROJECT_NAME} ${dependencies} Eigen3)

install(TARGETS
  floam_laser_processing_node
)
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()