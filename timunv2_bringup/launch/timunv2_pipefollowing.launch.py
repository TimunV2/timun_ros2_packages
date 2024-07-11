import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create LaunchDescription object
    ld = LaunchDescription()

    pipeline_detect_node = Node(
        package="timunv2_pipefollowing",
        executable="pipeline_detect_node"
    )
    pipeline_navigation_node = Node(
        package="timunv2_pipefollowing",
        executable="pipeline_navigation_node"
    )

    ld.add_action(pipeline_detect_node) #yolov8 detect
    ld.add_action(pipeline_navigation_node) #yolov8 detect

    return ld