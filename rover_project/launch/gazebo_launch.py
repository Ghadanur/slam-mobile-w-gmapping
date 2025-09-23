from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('rover_project')
    world_path = os.path.join(pkg_share, 'worlds', 'Warehouse.world')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    # if not os.path.exists(world_path):
    #     raise FileNotFoundError(f"❌ World file not found: {world_path}")
    
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    spawn_rover = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover', '-topic', 'robot_description'],
        output='screen'
    )

    # 🚀 Only spawn after Gazebo starts
    spawn_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gazebo,
            on_start=[spawn_rover],
        )
    )

    return LaunchDescription([
        gazebo,
        rsp,
        spawn_after_gazebo
    ])
