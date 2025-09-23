from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    return LaunchDescription([
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
