import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    camera_publisher_node_front = Node(
        package="timunv2_camera",
        executable="camera_publisher_node",
        name="camera_publisher_node_front",
        arguments=["/dev/video4","/camera_front"]
    )
    camera_publisher_node_bottom = Node(
        package="timunv2_camera",
        executable="camera_publisher_node",
        name="camera_publisher_node_bottom",
        arguments=["/dev/video1","/camera_bottom"]
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

    # ld.add_action(camera_publisher_node_front) #to publish front camera
    ld.add_action(camera_publisher_node_bottom) #to publish front camera
    ld.add_action(master_controller_node) #to select the final output of cmd velocity and set point
    ld.add_action(serial_node) #to convert and send the data to stm, and to read sensor data
    ld.add_action(ping_node) #to convert and send the data to stm, and to read sensor data
    ld.add_action(record_node) #to publish utilities for data record

    return ld