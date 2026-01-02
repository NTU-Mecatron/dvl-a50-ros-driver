#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    params = {
        'tcp_ip': '192.168.2.95',
        'tcp_port': 16171,
        'dvl_topic': 'velocity',
        'dead_reckoning_topic': 'dead_reckoning',
        'reset_dead_reckoning': 'reset_dead_reckoning',
        'calibrate_gyro': 'calibrate_gyro',
        'get_config': 'get_config',
        'turn_off': 'turn_off',
        'turn_on': 'turn_on',
        'toggle': 'toggle',
        'log_raw_data': False,
        'dvl_frame_id': 'dvl_link',
        'output_twist_stamped_topic': 'twist_stamped'
    }

    dvl_node = Node(
        package='dvl_a50_ros_driver',
        executable='publisher',
        name='dvl',
        namespace='dvl',
        output='screen',
        parameters=[params]
    )

    ld.add_action(dvl_node)

    return ld
