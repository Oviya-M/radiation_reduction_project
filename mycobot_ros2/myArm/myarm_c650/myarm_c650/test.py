#!/usr/bin/env python3

"""
This package requires `pymycobot`.
This script tests the API functionality.

Only works on Linux.
"""

import rclpy
from rclpy.node import Node
from math import pi
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from pymycobot.myarmc import MyArmC
import subprocess
import time


# 现行变换
def linear_transform(x):
    # Two known data points
    x1, y1 = -89.5, 0.022
    x2, y2 = 0, 0
    
    # Calculate slope (m) and intercept (c) of the line
    m = (y2 - y1) / (x2 - x1)
    c = y1 - m * x1
    
    # Apply linear transformation
    y = m * x + c
    
    return y

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('pub_data')  # ROS2节点初始化
        
        # 添加串口 Initialize the MyArmC object
        self.arm = MyArmC('/dev/ttyACM0', debug=False)
        
        # 创建发布者 Create publisher
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # 设置发布时间（频率） Set publishing timer
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        
        self.get_logger().info('JointStatePublisher node has been started')

    def timer_callback(self):
        # 创建关节状态信息 Create JointState message
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # 获取关节角度 Get joint angles
        angles = self.arm.get_joints_angle()
        gripper_angle = angles.pop(6)
        
        # 角度转弧度 Convert angles to radians
        angles_in_radians = [a / 180 * pi for a in angles]
        
        # 将夹爪实现现行变换 Apply linear transform to gripper angle
        transformed_gripper_angle = linear_transform(gripper_angle)
        angles_in_radians.append(transformed_gripper_angle)
        
        # Populate JointState message
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        joint_state.position = angles_in_radians
        joint_state.velocity = []
        joint_state.effort = []
        
        # 发布信息 Publish the message
        self.publisher_.publish(joint_state)
        self.get_logger().info('Message successfully published')

def main(args=None):
    rclpy.init(args=args)
    
    node = JointStatePublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()