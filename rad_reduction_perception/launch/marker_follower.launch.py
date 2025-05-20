from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rad_reduction_perception',
            executable='marker_to_robot',
            output='screen',
        ),
    ])
