import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    declare_video_device_front_arg = DeclareLaunchArgument(
        'video_device_front',
        default_value='/dev/video4',
        description='Path to the front camera video device (e.g., /dev/video0, /dev/video1, etc.)'
    )

    declare_video_device_bottom_arg = DeclareLaunchArgument(
        'video_device_bottom',
        default_value='/dev/video0',
        description='Path to the bottom camera video device (e.g., /dev/video0, /dev/video1, etc.)'
    )

    # Create LaunchDescription object
    ld = LaunchDescription()

    # Add the declared launch arguments
    ld.add_action(declare_video_device_front_arg)
    ld.add_action(declare_video_device_bottom_arg)

    camera_publisher_node_front = Node(
        package="timunv2_camera",
        executable="camera_publisher_node",
        name="camera_publisher_node_front",
        arguments=[LaunchConfiguration('video_device_front'), "/camera_front"]
    )
    camera_publisher_node_bottom = Node(
        package="timunv2_camera",
        executable="camera_publisher_node",
        name="camera_publisher_node_bottom",
        arguments=[LaunchConfiguration('video_device_bottom'), "/camera_bottom"]
    )

    master_controller_node = Node(
        package="timunv2_controller",
        executable="master_controller"
    )

    serial_node = Node(
        package="timunv2_serial",
        executable="serial_node"
    )
    record_node = Node(
        package="timunv2_serial",
        executable="record_node"
    )
    ping_node = Node(
        package="timunv2_serial",
        executable="ping_node"
    )

    display_data_node = Node(
        package="timunv2_serial",
        executable="display_data_node"
    )

    # ld.add_action(camera_publisher_node_front) #to publish front camera
    # ld.add_action(camera_publisher_node_bottom) #to publish front camera
    ld.add_action(master_controller_node) #to select the final output of cmd velocity and set point
    ld.add_action(serial_node) #to convert and send the data to stm, and to read sensor data
    # ld.add_action(ping_node) #to convert and send the data to stm, and to read sensor data
    ld.add_action(record_node) #to publish utilities for data record
    ld.add_action(display_data_node) #to publish utilities for data record

    return ld