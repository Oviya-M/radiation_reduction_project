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

def linear_transform(x):
    """
    Apply a linear transformation to the input value.
    
    Parameters:
    x (float): The input value.
    
    Returns:
    float: The transformed value.
    """
    # Two known data points
    x1, y1 = -89.5, 0.022
    x2, y2 = 0, 0
    
    # Calculate slope (m) and intercept (c) of the line
    m = (y2 - y1) / (x2 - x1)
    c = y1 - m * x1
    
    # Apply linear transformation
    y = m * x + c
    
    return y

def kill_node(node_name):
    """
    Kill a ROS2 node by its name.
    
    Parameters:
    node_name (str): The name of the node to kill.
    """
    try:
        # Get the list of all nodes
        result = subprocess.run(['ros2', 'node', 'list'], stdout=subprocess.PIPE, text=True)
        nodes = result.stdout.split()
        
        if node_name in nodes:
            # Get the node's PID using its name
            result = subprocess.run(['pgrep', '-f', node_name], stdout=subprocess.PIPE, text=True)
            pid = result.stdout.strip()
            
            if pid:
                # Kill the node by PID
                subprocess.run(['kill', pid])
                print(f"Node {node_name} with PID {pid} has been killed.")
            else:
                print(f"Node {node_name} not found.")
        else:
            print(f"Node {node_name} not found in active nodes.")
    except Exception as e:
        print(f"Failed to kill node {node_name}: {e}")

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('pub_data')  # ROS2节点初始化
        
        # Initialize the MyArmC object
        self.arm = MyArmC('/dev/ttyACM0', debug=False)
        
        # Create publisher
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        # Set publishing timer
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz
        
        self.get_logger().info('JointStatePublisher node has been started')

    def timer_callback(self):
        # Create JointState message
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Get joint angles
        angles = self.arm.get_joints_angle()
        gripper_angle = angles.pop(6)
        
        # Convert angles to radians
        angles_in_radians = [a / 180 * pi for a in angles]
        
        # Apply linear transform to gripper angle
        transformed_gripper_angle = linear_transform(gripper_angle)
        angles_in_radians.append(transformed_gripper_angle)
        
        # Populate JointState message
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        joint_state.position = angles_in_radians
        joint_state.velocity = []
        joint_state.effort = []
        
        # Publish the message
        self.publisher_.publish(joint_state)
        self.get_logger().info('Message successfully published')

def main(args=None):
    # Kill the joint_state_publisher node if it exists
    kill_node('joint_state_publisher')
    
    # Ensure the node has been terminated
    time.sleep(2)

    rclpy.init(args=args)
    
    node = JointStatePublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()