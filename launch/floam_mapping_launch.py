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

  bag_exec = ExecuteProcess(
    cmd=["ros2", "bag", "play", "-r", "0.85", "/workspaces/vscode_ros2_workspace/BTG_hesai_64_2023_11_09-12_56_42" , "--topics", "/pandar", "--clock"]
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
    rviz_node
  ])
