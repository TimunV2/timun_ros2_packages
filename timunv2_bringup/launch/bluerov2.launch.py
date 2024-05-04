import os
from launch import LaunchDescription
from launch_ros.actions import RosTimer
from launch.actions import ExecuteProcess
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

    gstreamer_pub_node = Node(
        package="timunv2_camera",
        executable="gstreamer_publisher_node"
    )

    master_controller_node = Node(
        package="timunv2_controller",
        executable="master_controller"
    )
    gui_node = ExecuteProcess(
        cmd=[['ros2 ','run ','timunv2_gui ','gui_node']],
        shell=True
    )

    mavlink_node = Node(
        package="timunv2_serial",
        executable="bluerov2_mavlink"
    )

    pipeline_detect_node = Node(
        package="timunv2_pipefollowing",
        executable="pipeline_detect_node"
    )
    
    pipeline_navigation_node = Node(
        package="timunv2_pipefollowing",
        executable="pipeline_navigation_node"
    )
    

    ld.add_action(joy_node) #to read pilot input via joy stick like button and analog sticks
    ld.add_action(joy_sub_node) #to convert button and axis to velocity and utility command
    ld.add_action(master_controller_node) #to select the final output of cmd velocity and set point
    ld.add_action(mavlink_node) #to convert and send the data to stm, and to read sensor data
    ld.add_action(gstreamer_pub_node) #to convert and send the data to stm, and to read sensor data
    ld.add_action(gui_node) #to open GUI
    # ld.add_action(RosTimer(period=10.0,actions=[pipeline_detect_node])) #to convert and send the data to stm, and to read sensor data
    # ld.add_action(RosTimer(period=10.0,actions=[pipeline_navigation_node])) #to convert and send the data to stm, and to read sensor data
    # ld.add_action(pipeline_navigation_node) #to convert and send the data to stm, and to read sensor data

    return ld