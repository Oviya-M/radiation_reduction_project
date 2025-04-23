import os
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # 获取包路径
    myarm_m750_description_pkg = FindPackageShare('myarm_m750').find('myarm_m750')
    # myarm_c650_description_pkg = FindPackageShare('myarm_c650').find('myarm_c650')
    mycobot_description_pkg = FindPackageShare('mycobot_description').find('mycobot_description')

    # 定义 URDF 文件路径
    urdf_m750 = os.path.join(mycobot_description_pkg, 'urdf','myarm_m750', 'myarm_m750.urdf.xacro')
    urdf_c650 = os.path.join(mycobot_description_pkg, 'urdf','myarm_c650', 'myarm_c650.urdf.xacro')

    # 定义 RViz 配置文件路径
    rviz_config_file = os.path.join(myarm_m750_description_pkg, 'config', 'myarm_m750_with_myarm_c650.rviz')

    ns1_arg = LaunchConfiguration('ns1')
    ns2_arg = LaunchConfiguration('ns2')
    
    return LaunchDescription([
        # 定义是否使用仿真时间参数
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        # myarm_m750 的 robot_state_publisher 节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='myarm_m750',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'), 
                'robot_description': Command(['xacro ', urdf_m750])
            }],
            output='screen',
        ),
        
        # myarm_m750 的 joint_state_publisher_gui 节点
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace='myarm_m750',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
        
        # myarm_c650 的 robot_state_publisher 节点
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='myarm_c650',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'), 
                'robot_description': Command(['xacro ', urdf_c650])
            }],
            output='screen',
        ),
        
        # myarm_c650 的 joint_state_publisher_gui 节点
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace='myarm_c650',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
        
        # RViz 节点
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
        
        # 静态 TF 发布节点
        DeclareLaunchArgument(
            'ns1',
            default_value='myarm_m750',
            description='Namespace 1'
        ),
        DeclareLaunchArgument(
            'ns2',
            default_value='myarm_c650',
            description='Namespace 2'
        ),

        # Transform for ns2
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher1_ns2',
            output='screen',
            arguments=['0', '-0.5', '0', '0', '0', '0', '/map', [ns2_arg, '/base']]
        ),
        # Transform for ns1
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher1_ns1',
            output='screen',
            arguments=['0', '0.5', '0', '0', '0', '0', '/map', [ns1_arg, '/base']]
        )
    ])


