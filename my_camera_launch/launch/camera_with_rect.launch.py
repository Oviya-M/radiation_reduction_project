from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'camera_name': 'default_cam',
                'pixel_format': 'yuyv',
                'camera_info_url': 'file:///home/macs/.ros/camera_info/default_cam.yaml'
            }],
            remappings=[
                ('/image_raw', '/image_raw'),
                ('/camera_info', '/camera_info')
            ]
        ),
        Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc',
            remappings=[
                ('image_raw', '/image_raw'),
                ('camera_info', '/camera_info'),
                ('image', '/image')
            ]
        ),
    ])
