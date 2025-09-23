from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        #Include Gazebo and world launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rover_project'),
                    'launch',
                    'warehouse_rover.launch.py'
                ])
            )
        ),
        # Include Gazebo and robot launch
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('rover_project'),
        #             'launch',
        #             'gazebo_launch.py'
        #         ])
        #     )
        # ),

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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rover_project'),
                    'launch',
                    'slam_gmapping.launch.py'
                ])
            )
        ),
    ]) 