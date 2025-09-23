from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_project')
    urdf_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    return LaunchDescription([

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        ),
    ])