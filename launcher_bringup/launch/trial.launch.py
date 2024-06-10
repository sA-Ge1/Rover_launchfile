from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
# chnage in code
def generate_launch_description():
    ld = LaunchDescription()
    
    # Nodes for static transforms
    N1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='N1',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'laser']
    )
    
    N2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='N2',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_node'
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/humble/share/slam_toolbox/launch/online_async_launch.py'
        ])
    )

    ld.add_action(N1)
    ld.add_action(N2)
    ld.add_action(navigation_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(teleop_node)
    
    return ld
