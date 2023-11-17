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
##PYBIND 11 wrapper stuff starts here
set(CMAKE_CXX_STANDARD 20)
set(PYBIND11_PYTHON_VERSION 3.6)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
find_package(Python3 REQUIRED COMPONENTS Interpreter Development ${_pybind11_quiet}) 
find_package(pybind11 REQUIRED)



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
# ament_export_include_directories(include)

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
  pybind11
)



add_executable(floam_laser_processing_node 
  src/preprocessing/laserProcessing.cpp 
  src/preprocessing/laserProcessingClass.cpp
  src/preprocessing/laser_processing_node.cpp 
  src/lidar.cpp)
ament_target_dependencies(floam_laser_processing_node  ${dependencies} )
target_link_libraries(floam_laser_processing_node ${CERES_LIBRARIES})
target_include_directories(floam_laser_processing_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)     


add_executable(floam_odom_estimation_node 
  src/odometry/odomEstimation.cpp 
  src/lidarOptimization.cpp
  src/odometry/odom_estimation_node.cpp 
  src/lidar.cpp 
  src/odometry/odomEstimationClass.cpp)
ament_target_dependencies(floam_odom_estimation_node ${dependencies} Eigen3)
target_link_libraries(floam_odom_estimation_node ${EIGEN3_LIBRARIES}  ${PCL_LIBRARIES} ${CERES_LIBRARIES})


add_executable(floam_laser_mapping_node 
  src/mapping/laserMapping.cpp 
  src/mapping/laserMappingClass.cpp
  src/mapping/laser_mapping_node.cpp
  src/lidar.cpp)
ament_target_dependencies(floam_laser_mapping_node  ${dependencies} Eigen3)
target_link_libraries(floam_laser_mapping_node ${EIGEN3_LIBRARIES}  ${PCL_LIBRARIES} ${CERES_LIBRARIES})


##HACK : ament_target_dependencies cannot declare its deps as private, which is needed for pybind11.. so copypaste
function(ament_target_dependencies_private target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    set(definitions "")
    set(include_dirs "")
    set(libraries "")
    set(link_flags "")
    foreach(package_name ${ARGN})
      if(NOT ${${package_name}_FOUND})
        message(FATAL_ERROR "ament_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      list_append_unique(definitions ${${package_name}_DEFINITIONS})
      list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})
      list(APPEND libraries ${${package_name}_LIBRARIES})
      list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
    endforeach()
    target_compile_definitions(${target}
      PUBLIC ${definitions})
    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_include_directories(${target}
      PUBLIC ${ordered_include_dirs})
    ament_libraries_deduplicate(unique_libraries ${libraries})
    target_link_libraries(${target} PRIVATE
      ${unique_libraries})
    foreach(link_flag IN LISTS link_flags)
      set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
    endforeach()
  endif()
endfunction()


###############################################################################################
#LIDAR CLASS 
pybind11_add_module(${PROJECT_NAME}_lidar pybind/lidar_parameter.cpp src/lidar.cpp )

set_target_properties(${PROJECT_NAME}_lidar PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")


target_include_directories(${PROJECT_NAME}_lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )



ament_target_dependencies_private(
  ${PROJECT_NAME}_lidar
  "objectdetector_msgs"
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  pybind11::module
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}_lidar PRIVATE "FAST_ROS_PI_BUILDING_LIBRARY")
#target_link_libraries(${PROJECT_NAME} PRIVATE  ${dependencies} )


############################################################################################
#PRE-PROCESSING PYTHON 

pybind11_add_module(${PROJECT_NAME}_preprocessing pybind/preprocessing.cpp src/preprocessing/laserProcessingClass.cpp src/lidar.cpp)

set_target_properties(${PROJECT_NAME}_preprocessing PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")


target_include_directories(${PROJECT_NAME}_preprocessing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )



ament_target_dependencies_private(
  ${PROJECT_NAME}_preprocessing
  "objectdetector_msgs"
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  pybind11::module
)

#target_compile_definitions(${PROJECT_NAME}_preprocessing PRIVATE "FAST_ROS_PI_BUILDING_LIBRARY")
#ament_target_dependencies(${PROJECT_NAME}_preprocessing  ${dependencies} )
target_link_libraries(${PROJECT_NAME}_preprocessing PRIVATE ${CERES_LIBRARIES}  ${PCL_LIBRARIES})


############################################################################################
############################################################################################
#ODOMETRY-PROCESSING PYTHON 

pybind11_add_module(${PROJECT_NAME}_odometry pybind/odometry.cpp )

set_target_properties(${PROJECT_NAME}_odometry PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")

target_include_directories(${PROJECT_NAME}_odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

ament_target_dependencies_private(
  ${PROJECT_NAME}_odometry
  "objectdetector_msgs"
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  pybind11::module
)

target_compile_definitions(${PROJECT_NAME}_odometry PRIVATE "FAST_ROS_PI_BUILDING_LIBRARY")

#########################################################################################
############################################################################################
#MAPPING-PROCESSING PYTHON 

pybind11_add_module(${PROJECT_NAME}_mapping pybind/odometry.cpp )
set_target_properties(${PROJECT_NAME}_mapping PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")
target_include_directories(${PROJECT_NAME}_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies_private(
  ${PROJECT_NAME}_mapping
  "objectdetector_msgs"
  "rclcpp"
  "sensor_msgs"
  "pcl_conversions"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  pybind11::module
)
target_compile_definitions(${PROJECT_NAME}_mapping PRIVATE "FAST_ROS_PI_BUILDING_LIBRARY")

#########################################################################################



install(TARGETS 
  floam_laser_processing_node 
  floam_odom_estimation_node 
  floam_laser_mapping_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch rviz params
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(${dependencies} pybind11)
ament_package()


