import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    joy_node = Node(
        package="joy",
        executable="joy_node"
    )

    joy_sub_node = Node(
        package="timunv2_controller",
        executable="joy_sub_node"
    )

    master_controller_node = Node(
        package="timunv2_controller",
        executable="master_controller"
    )

    serial_node = Node(
        package="timunv2_serial",
        executable="serial_node"
    )

    ld.add_action(joy_node) #to read joy stick input like button and analog
    ld.add_action(joy_sub_node) #to convert button and axis to velocity and utility command
    ld.add_action(master_controller_node) #to select the final output of cmd velocity and set point
    ld.add_action(serial_node) #to convert and send the data to stm, and to read sensor data

    return ld