from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
  params = join(
    get_package_share_directory("floam"), "params", "floam_params.yaml"
  )

  laser_processing_node = Node(
    package="floam",
    executable="floam_laser_processing_node",
    name="laser_processing_node",
    parameters=[params]
  )

  odom_estimation_node = Node(
    package="floam",
    executable="floam_odom_estimation_node",
    name="odom_estimation_node",
    parameters=[params]
  )

  laser_mapping_node = Node(
    package="floam",
    executable="floam_laser_mapping_node",
    name="laser_mapping_node",
    parameters=[params]
  )

  
  return LaunchDescription([
    laser_processing_node,
    odom_estimation_node,
    laser_mapping_node,
    
  ])
