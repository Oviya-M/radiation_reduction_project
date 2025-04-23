from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rad_reduction_perception',
            executable='publisher',
            name='rad_publisher',
            output='screen'
        ),
        Node(
            package='rad_reduction_perception',
            executable='my_subscriber',
            name='rad_subscriber',
            output='screen'
        ),
    ])
