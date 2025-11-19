from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_project')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')
    
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
    # Launch Empty World in Gazebo Classic
        ExecuteProcess(
            cmd=[
                "gazebo",
                "--verbose",
                "-s", "libgazebo_ros_factory.so",
                "/usr/share/gazebo-11/worlds/empty.world"
            ],
            output="screen"
        ),
      

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
        
        # Spawn Rover in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'rover', '-topic', 'robot_description'],
            output='screen'
        ),

        # Publish TF for URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
               'robot_description': robot_desc,
               'use_sim_time': True
        }]
        ),
    ])
