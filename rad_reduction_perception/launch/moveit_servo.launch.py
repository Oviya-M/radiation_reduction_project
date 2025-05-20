#!/usr/bin/env python3
"""
Bring up:
  • robot_state_publisher              (URDF → TF)
  • joint_state_broadcaster + traj ctr
  • MoveIt² move_group
  • MoveIt Servo node (listening for /servo_node/pose_target)
  • RViz2 with the MoveIt config
Pass the serial port & baud on the CLI, e.g.:
  ros2 launch mycobot_moveit_config moveit_servo.launch.py \
         port:=/dev/ttyACM1 baud:=115200
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mycobot_moveit_config')

    port_arg  = DeclareLaunchArgument('port',  default_value='/dev/ttyACM0')
    baud_arg  = DeclareLaunchArgument('baud',  default_value='115200')
    servo_cfg = os.path.join(pkg_share, 'config', 'ur_servo.yaml')
    controllers_yaml = os.path.join(pkg_share, 'config', 'ros2_controllers.yaml')


    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(
            os.path.join(pkg_share, 'config', 'firefighter.urdf.xacro')
               ).read()},        # description loaded as plain text
                   {'use_sim_time': False}],
        output='screen')

    joint_state_broad = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster'],
        output='screen')

    traj_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_trajectory_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen')

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[os.path.join(pkg_share, 'config', 'moveit_controllers.yaml'),
                    {'allow_trajectory_execution': True}],
        output='screen')

    servo = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        parameters=[servo_cfg],
        output='screen')

    # (Optional) RViz pre-configured for planning
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'moveit.rviz')],
        output='screen')

    return LaunchDescription([
        port_arg, baud_arg,
        rsp,
        joint_state_broad,
        traj_controller,
        move_group,
        servo,
        rviz,
    ])
