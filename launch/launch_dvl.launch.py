#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    params = {
        'tcp_ip': '192.168.2.95',
        'tcp_port': 16171,
        'dvl_raw_topic': 'raw_data',
        'dvl_topic': 'original_data',
        'dead_reckoning_topic': 'dead_reckoning',
        'reset_dead_reckoning': 'reset_dead_reckoning',
        'output_twist_stamped_topic': 'twist_stamped',
        'calibrate_gyro': 'calibrate_gyro',
        'get_config': 'get_config',
        'turn_off': 'turn_off',
        'turn_on': 'turn_on',
        'toggle': 'toggle',
        'log_raw_data': False,
        'dvl_frame_id': 'dvl_link',
        'use_fom_to_compute_covariance': True,      # Set to False to use custom covariances
        'linear_vel_var_x': 0.01,       # Custom X-axis velocity variance (m²/s²)
        'linear_vel_var_y': 0.01,       # Custom Y-axis velocity variance (m²/s²)
        'linear_vel_var_z': 0.01,       # Custom Z-axis velocity variance (m²/s²)
    }

    dvl_node = Node(
        package='dvl_a50_ros_driver',
        executable='publisher',
        name='publisher',
        namespace='dvl',
        output='screen',
        parameters=[params]
    )

    dvl_repub_node = Node(
        package='dvl_a50_ros_driver',
        executable='dvl_republisher',
        name='dvl_republisher',
        namespace='dvl',
        output='screen',
        parameters=[params]
    )

    ld.add_action(dvl_node)
    ld.add_action(dvl_repub_node)

    return ld
