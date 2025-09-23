from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get rover_project paths
    pkg_share = get_package_share_directory('rover_project')
    world_path = os.path.join(pkg_share, 'worlds', 'moon.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    # Safety check: world must exist
    if not os.path.exists(world_path):
        raise FileNotFoundError(f"❌ World file not found: {world_path}")
    
    # Read URDF
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Start Gazebo with moon.world
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world_path,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Robot State Publisher
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

        # Spawn rover into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'rover',
                '-topic', 'robot_description'
            ],
            output='screen'
        )
    ])