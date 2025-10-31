from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map', default='')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file to load (leave empty if using SLAM)'
    )

    # Paths
    bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_launch = os.path.join(bringup_dir, 'launch', 'bringup_launch.py')

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_launch = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    # Include Nav2 bringup
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file
        }.items()
    )

    # If no map is provided → also start slam_toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map_yaml_file)

    # Conditionally include slam
    from launch.conditions import IfCondition, UnlessCondition
    ld.add_action(slam)   # run SLAM if map_yaml_file is empty
    ld.add_action(nav2)

    return ld
