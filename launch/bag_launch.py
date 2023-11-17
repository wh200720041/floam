from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from os.path import join


def generate_launch_description():
  

  bag_exec = ExecuteProcess(
    cmd=["ros2", "bag", "play", "-r", "0.85", "/home/chris/SLAM/vscode_ros2_workspace/BTG_hesai_64_2023_11_09-12_56_42" , "--topics", "/pandar", "--clock"]
  )

  
  return LaunchDescription([
    bag_exec
  ])
