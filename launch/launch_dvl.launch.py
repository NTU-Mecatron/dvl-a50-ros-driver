#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file for DVL A50 ROS2 driver.
    
    Launches two nodes:
    1. publisher: Communicates with DVL hardware via TCP/IP
    2. dvl_republisher: Converts raw DVL data to ROS2 standard messages
    
    Parameters are loaded from dvl_params.yaml.
    """
    ld = LaunchDescription()

    # Get the path to the parameter file
    pkg_share = get_package_share_directory('dvl_a50_ros_driver')
    params_file = os.path.join(pkg_share, 'params', 'dvl_params.yaml')

    # DVL Publisher Node - Hardware interface
    dvl_node = Node(
        package='dvl_a50_ros_driver',
        executable='publisher',
        name='raw_data_publisher',
        namespace='dvl',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
    )

    # DVL Republisher Node - Data conversion
    dvl_repub_node = Node(
        package='dvl_a50_ros_driver',
        executable='dvl_republisher',
        name='twist_republisher',
        namespace='dvl',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
    )

    ld.add_action(dvl_node)
    ld.add_action(dvl_repub_node)

    return ld
