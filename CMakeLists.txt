cmake_minimum_required(VERSION 2.8.3)
project(floam_g2o)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf
  eigen_conversions
)


set(EIGEN3_FOUND true)
find_package(Eigen3 REQUIRED)
IF(EIGEN3_FOUND) 
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
MESSAGE("EIGEN3 [OK]:" ${EIGEN3_INCLUDE_DIR})
ENDIF(EIGEN3_FOUND)


find_package(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

set(3RDPARTY_DIR /home/cc/workspace/3rdparty)
find_package(Ceres REQUIRED PATHS ${3RDPARTY_DIR}/ceres-1.14 )

message(STATUS "ceres: " ${GERES_LIBRARIES})
# find_package(Ceres REQUIRED)

message(STATUS "pcl dir: ${PCL_INCLUDE_DIRS}")

set(G2O_INCLUDE_DIRS ${3RDPARTY_DIR}/g2o/g2o2017_CSparse/include)
set(G2O_LIB_DIR ${3RDPARTY_DIR}/g2o/g2o2017_CSparse/lib) 
SET(G2O_LIBRARIES g2o_cli g2o_core g2o_csparse_extension g2o_ext_freeglut_minimal 
             g2o_incremental g2o_interactive g2o_interface g2o_opengl_helper 
             g2o_parser g2o_simulator g2o_solver_cholmod g2o_solver_csparse 
             g2o_solver_dense g2o_solver_eigen g2o_solver_pcg g2o_solver_slam2d_linear 
             g2o_solver_structure_only g2o_stuff g2o_types_data g2o_types_icp 
             g2o_types_sba g2o_types_sclam2d g2o_types_sim3 g2o_types_slam2d
             g2o_types_slam2d_addons g2o_types_slam3d g2o_types_slam3d_addons g2o_viewer)
IF(G2O_FOUND)
    message("G2O lib is found:" ${G2O_INCLUDE_DIRS}) 
    message("G2O lib:" ${G2O_LIBRARIES})
ENDIF(G2O_FOUND)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
  ${G2O_LIB_DIR}
)


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL Ceres 
  INCLUDE_DIRS include
)



add_executable(${PROJECT_NAME}_processing_node src/laserProcessingNode.cpp src/laserProcessingClass.cpp src/lidar.cpp)
target_link_libraries(${PROJECT_NAME}_processing_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(${PROJECT_NAME}_estimation_node src/odomEstimationNode.cpp src/lidarOptimization.cpp src/lidar.cpp src/odomEstimationClass.cpp)
target_link_libraries(${PROJECT_NAME}_estimation_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${G2O_LIBRARIES})

add_executable(${PROJECT_NAME}_mapping_node src/laserMappingNode.cpp src/laserMappingClass.cpp src/lidar.cpp)
target_link_libraries(${PROJECT_NAME}_mapping_node ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(recordTrajNode src/recordTrajNode.cpp)
target_link_libraries(recordTrajNode ${catkin_LIBRARIES})