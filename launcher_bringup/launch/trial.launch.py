from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
    namespace = ''

    # Path to the yaml configuration file
    config = '/home/sage/ros/src/launcher_bringup/config/mapper_params_online_async.yaml'
    
    # Nodes for static transforms
    N0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='N0',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py'
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Used nodes
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
    
    N3 = Node(
        package='hal_diff',
        executable='diff'
    )
    
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_node',
        output = 'screen',
        namespace = namespace,
        prefix='gnome-terminal --'
    )

    yaml_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value= config,
        description='Full path to the ROS2 parameters file to use for the SLAM toolbox node'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        output='screen',
        parameters=[LaunchConfiguration('slam_params_file')]
    )

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Scan mode for the SLLidar'
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/sage/slidder_ws/src/sllidar_ros2/launch/view_sllidar_a1_launch.py'
        ]),
        launch_arguments={'scan_mode': LaunchConfiguration('scan_mode')}.items()
    )

    ld.add_action(scan_mode_arg)
    ld.add_action(sllidar_launch)

    ld.add_action(N0)
    ld.add_action(N1)
    ld.add_action(N2)
    ld.add_action(N3)
    ld.add_action(teleop_node)
    ld.add_action(yaml_file_arg)
    ld.add_action(slam_toolbox_node)
    #ld.add_action(navigation_launch)
    
    
    return ld