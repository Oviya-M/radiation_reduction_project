from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    rvizconfig = LaunchConfiguration('rvizconfig')
    gui = LaunchConfiguration('gui')
    ns1 = LaunchConfiguration('ns1')
    ns2 = LaunchConfiguration('ns2')

    rviz_config_default = os.path.join(get_package_share_directory('myarm_m750'), 'config', 'myarm_m750_with_myarm_c650.rviz')
    urdf_m750_path = os.path.join(get_package_share_directory('mycobot_description'), 'urdf', 'myarm_m750', 'myarm_m750.urdf.xacro')
    urdf_c650_path = os.path.join(get_package_share_directory('mycobot_description'), 'urdf', 'myarm_c650', 'myarm_c650.urdf.xacro')

    return LaunchDescription([
        DeclareLaunchArgument('rvizconfig', default_value=TextSubstitution(text=rviz_config_default)),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('ns1', default_value='myarm_m750'),
        DeclareLaunchArgument('ns2', default_value='myarm_c650'),
        
        # myarm_m750 group
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns1,
            parameters=[{
                'robot_description': Command(['xacro ', urdf_m750_path, ' ns:=', ns1]),
                'publish_frequency': 50.0,
                'tf_prefix': ns1
            }],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=ns1,
            parameters=[{'use_gui': gui}],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher1',
            namespace=ns1,
            arguments=['0', '0.5', '0', '0', '0', '0', '/map', [LaunchConfiguration('ns1'), '/base']],
            output='screen'
        ),

        # myarm_c650 group
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns2,
            parameters=[{
                'robot_description': Command(['xacro ', urdf_c650_path, ' ns:=', ns2]),
                'publish_frequency': 50.0,
                'tf_prefix': ns2
            }],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            namespace=ns2,
            parameters=[{'use_gui': gui}],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher2',
            namespace=ns2,
            arguments=['0', '-0.5', '0', '0', '0', '0', '/map', [LaunchConfiguration('ns2'), '/base']],
            output='screen'
        ),

        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rvizconfig],
            output='screen'
        )
    ])