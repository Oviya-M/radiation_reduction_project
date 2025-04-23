#!/usr/bin/env python3
# encoding:utf-8
# 读取机器人关节角数据，发送给RVIZ

from pymycobot import MyArmM
import rclpy
from rclpy.node import Node
from math import pi
from time import sleep
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import subprocess

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('pub_data')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz
        self.mam = MyArmM('/dev/ttyACM0', debug=False)
        
        # 放松关节
        for i in range(8):
            self.mam.set_servo_enabled(i, 0)
            sleep(0.2)
        
        # 尝试关闭节点
        self.shutdown_ros_node('joint_state_publisher_gui')
        self.get_logger().info("已成功杀死节点")
        
    def shutdown_ros_node(self, node_name):
        try:
            subprocess.run(['ros2', 'node', 'kill', node_name])
            self.get_logger().info(f"Node {node_name} has been shutdown.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Error: {e}")
    
    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        angles = self.mam.get_joints_angle()
        gripper_angle = angles.pop(6)
        gripper_angle /= -3500
        # angle: 弧度 angles: 角度
        angle = [a / 180 * pi for a in angles]
        angle.append(gripper_angle)
        self.get_logger().info(str(angle))
        joint_state.position = angle
        joint_state.effort = []
        joint_state.velocity = []
        self.pub.publish(joint_state)
        self.get_logger().info('消息成功发布')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()