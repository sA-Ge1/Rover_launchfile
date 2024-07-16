from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                'publish_tf': True,
                'frequency': 30.0,
                'imu0': 'imu/data',
                'imu0_config': [
                    False, False, False,
                    True, True, True,
                    True, True, True,
                    False, False, False,
                    False, False, False
                ],
                'imu0_queue_size': 5,
                'imu0_differential': False,
                'imu0_relative': True,
            }]
        )
    ])
