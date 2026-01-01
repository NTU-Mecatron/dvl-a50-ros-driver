#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    params = {
        'tcp_ip': '192.168.2.95',
        'tcp_port': 16171,
        'dvl_topic': '/dvl/velocity',
        'dead_reckoning_topic': '/dvl/dead_reckoning',
        'reset_dead_reckoning': '/dvl/reset_dead_reckoning',
        'calibrate_gyro': '/dvl/calibrate_gyro',
        'get_config': '/dvl/get_config',
        'turn_off': '/dvl/turn_off',
        'turn_on': '/dvl/turn_on',
        'toggle': '/dvl/toggle',
        'log_raw_data': False,
        'dvl_frame_id': 'auv/dvl_link',
        'output_twist_stamped_topic': 'dvl/twist_stamped'
    }

    dvl_node = Node(
        package='dvl_a50_ros_driver',
        executable='publisher',
        name='dvl',
        output='screen',
        parameters=[params]
    )

    dvl_repub_node = Node(
        package='dvl_a50_ros_driver',
        executable='dvl_republisher',
        name='dvl_republisher',
        output='screen',
        parameters=[params]
    )

    ld.add_action(dvl_node)
    ld.add_action(dvl_repub_node)

    return ld
