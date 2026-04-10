#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """
    Launch file for DVL A50 ROS2 driver.
    
    Launches two composable nodes in one process:
    1. dvl::RawJsonPublisher: Communicates with DVL hardware via TCP/IP
    2. dvl::TwistPublisher: Converts raw DVL data to ROS2 standard messages
    
    Parameters are loaded from dvl_params.yaml.
    """
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='dvl',
        description='Namespace for the DVL composable nodes',
    )
    ns = LaunchConfiguration('namespace')

    # Get the path to the parameter file
    params_dir = PathJoinSubstitution([FindPackageShare('dvl_a50_ros_driver'), 'params'])
    params_file = PathJoinSubstitution([params_dir, 'dvl_params.yaml'])

    ipc_arg = {'use_intra_process_comms': True}

    # DVL Publisher component - Hardware interface
    dvl_component = ComposableNode(
        package='dvl_a50_ros_driver',
        plugin='dvl::RawJsonPublisher',
        name='raw_data_publisher',
        namespace=ns,
        parameters=[params_file],
        extra_arguments=[ipc_arg],
    )

    # DVL Republisher component - Data conversion
    dvl_repub_component = ComposableNode(
        package='dvl_a50_ros_driver',
        plugin='dvl::TwistPublisher',
        name='twist_republisher',
        namespace=ns,
        parameters=[params_file],
        extra_arguments=[ipc_arg],
    )

    dvl_container = ComposableNodeContainer(
        name='dvl_container',
        namespace=ns,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            dvl_component,
            dvl_repub_component,
        ],
    )

    return LaunchDescription([
        namespace_arg,
        dvl_container,
    ])
