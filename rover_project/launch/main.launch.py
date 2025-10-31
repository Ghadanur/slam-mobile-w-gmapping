from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #bringup_dir = get_package_share_directory('nav2_bringup')
    config_dir = os.path.join(get_package_share_directory('rover_project'), 'config')
    map_file = os.path.join(config_dir, 'warehouse_map.yaml')
    params_file = os.path.join(get_package_share_directory('rover_project'), 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # #Include Nav2 and world launch
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('rover_project'),
        #             'launch',
        #             'bringup_launch.py'
        #         ])
        #     )
        # ),

        #Include Gazebo and world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rover_project'),
                    'launch',
                    'warehouse_rover.launch.py'
                ])
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true'
            }.items()
        ),

        # Include RViz launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rover_project'),
                    'launch',
                    'display.launch.py'
                ])
            )
        ),

        # Include SLAM gmapping launch
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('rover_project'),
        #             'launch',
        #             'slam_gmapping.launch.py'
        #         ])
        #     )
        # ),

        # Node(
        #     package='rover_project',
        #     executable='serial_nodes',
        #     name='rover_project',
        #     output='screen'
        # ),
        #  # RPLIDAR node
        # Node(
        #     package="rplidar_ros",
        #     executable="rplidar_composition",
        #     name="rplidar",
        #     output="screen",
        #     parameters=[{
        #         "serial_port": "/dev/ttyUSB1",
        #         "serial_baudrate": 115200,  # For A1/A2
        #         "frame_id": "laser_frame",
        #         "inverted": False,
        #         "angle_compensate": True
        #     }]
        # ),
        # Node(
        #     package='rover_project',
        #     executable='obstacle_avoidance',
        #     name='obstacle_avoidance',
        #     output='screen'
        # ),

     ]) 