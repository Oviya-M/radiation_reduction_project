#!/usr/bin/env python3
# encoding:utf-8
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from math import pi
import subprocess
import time
import datetime
import copy
from pymycobot import MyArmM, MyArmC

def shutdown_ros_node(node_name):
    try:
        subprocess.run(['ros2', 'node', 'kill', node_name])
        print(f"Node {node_name} has been shutdown.")
    except subprocess.CalledProcessError as e:
        print(f"Error: {e}")

def linear_transform(x):
    x1, y1 = -89.5, 0.022
    x2, y2 = 0, 0
    m = (y2 - y1) / (x2 - x1)
    c = y1 - m * x1
    y = m * x + c
    return y

class CombinedControl(Node):

    def __init__(self):
        super().__init__('combined_control')

        # 关闭节点
        shutdown_ros_node('myarm_m/joint_state_publisher_gui')
        shutdown_ros_node('myarm_c650/joint_state_publisher_gui')

        self.pub_m = self.create_publisher(JointState, 'myarm_m/joint_states', 10)
        self.pub_c = self.create_publisher(JointState, 'myarm_c650/joint_states', 10)

        self.timer = self.create_timer(0.02, self.timer_callback)

        self.joint_state = JointState()

        # 初始化 MyArmM 和 MyArmC 对象
        self.myarm_m = MyArmM('/dev/ttyACM1', debug=False)
        self.myarm_c = MyArmC('/dev/ttyACM0', debug=False)

    def timer_callback(self):
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.name = ['joint1', 'joint2', 'joint3','joint4', 'joint5', 'joint6','gripper']
        
        # Simulate getting joint angles
        anglesc = self.myarm_c.get_joints_angle()
        anglesm = copy.deepcopy(anglesc)
        
        gripper_angle_c_real = anglesc.pop(6)
        angle_c = [a/180*pi for a in anglesc]
        gripper_angle_c_sim = linear_transform(gripper_angle_c_real)
        angle_c.append(gripper_angle_c_sim)
        
        gripper_angle_c_real = anglesm.pop(6)
        gripper_angle_m_sim = gripper_angle_c_sim/0.022*0.0345
        angle_m = [a*pi/180 for a in anglesm]
        angle_m.append(gripper_angle_m_sim)
        angle_m[2]*=-1
        
        self.joint_state.position = angle_m
        self.pub_m.publish(self.joint_state)
        
        current_time1 = datetime.datetime.now()
        
        self.joint_state.position = angle_c
        self.pub_c.publish(self.joint_state)
        
        current_time2 = datetime.datetime.now()
        
        gripper_angle_m_sim = angle_m.pop(6)
        gripper_angle_m_real = gripper_angle_m_sim*(-3500)
        angle_m = [a*180/pi for a in angle_m]
        angle_m.append(gripper_angle_m_real)
        current_time = current_time2-current_time1
        print(angle_m)
        self.get_logger().info('消息成功发布')
        self.get_logger().info(current_time)

def main(args=None):
    rclpy.init(args=args)
    combined_control = CombinedControl()
    rclpy.spin(combined_control)
    combined_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()