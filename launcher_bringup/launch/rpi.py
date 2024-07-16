from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # Set ROS_DOMAIN_ID environment variable
    ros_domain_id = SetEnvironmentVariable('ROS_DOMAIN_ID', '7')

    # Path to the yaml configuration file
    config = '/home/astra/ros_ws/src/hall_driver_bringup/config/mapper_params_online_async.yaml'

    yaml_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=config,
        description='Full path to the ROS2 parameters file to use for the SLAM toolbox node'
    )

    N0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='N0',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

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

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='Standard',
        description='Scan mode for the SLLidar'
    )

    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(['/home/astra/rplidar_ws/src/sllidar_ros2/launch/sllidar_a1_launch.py']),
        launch_arguments={'scan_mode': LaunchConfiguration('scan_mode')}.items()
    )

    # Command to run micro_ros_agent
    micro_ros_agent_node = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial', '--dev', '/dev/ttyACM0'],
        output='screen'
    )

    ld.add_action(ros_domain_id)
    ld.add_action(scan_mode_arg)
    ld.add_action(sllidar_launch)
    ld.add_action(yaml_file_arg)
    ld.add_action(N0)
    ld.add_action(N1)
    ld.add_action(N2)
    ld.add_action(micro_ros_agent_node) 

    return ld
