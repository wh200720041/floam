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
    executable="laser_processing_node",
    name="laser_processing_node",
    parameters=[params]
  )

  odom_estimation_node = Node(
    package="floam",
    executable="odom_estimation_node",
    name="odom_estimation_node",
    parameters=[params]
  )

  laser_mapping_node = Node(
    package="floam",
    executable="laser_mapping_node",
    name="laser_mapping_node",
    parameters=[params]
  )

  bag_exec = ExecuteProcess(
    cmd=["ros2", "bag", "play", "-r", "0.5", "/data/Kitti/raw/2011_09_30_0018"]
  )

  tf_node = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="world2map_tf",
    arguments=['0', '0', '0', '0', '0', '0', "world", "map"]
  )

  rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    arguments=["-d", join(get_package_share_directory("floam"), "rviz/", "floam_mapping.rviz")]
  )

  return LaunchDescription([
    laser_processing_node,
    odom_estimation_node,
    laser_mapping_node,
    bag_exec,
    tf_node,
    rviz_node,
  ])
