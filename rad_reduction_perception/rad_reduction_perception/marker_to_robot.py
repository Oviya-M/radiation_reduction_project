#!/usr/bin/env python3
# marker_to_robot.py – moves the gripper 5 cm above ArUco ID 42

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
#from moveit_commander import MoveGroupCommander as MoveGroupInterface

MARKER_ID = 42
MARKER_TOPIC = "/aruco/poses"
ROBOT_ROOT = "base_link"
EEF_LINK   = "tool0"
OFFSET_Z   = 0.05   # 0.05 m = 5 cm

class MarkerFollower(Node):
    def __init__(self):
        super().__init__("marker_follower")
        self.tf_buf = Buffer()
        TransformListener(self.tf_buf, self)
        self.moveit = MoveGroupInterface("arm_group", EEF_LINK, self)
        self.sub = self.create_subscription(PoseStamped, MARKER_TOPIC, self.cb, 10)
        self.get_logger().info("Waiting for marker…")

    def cb(self, msg):
        if msg.header.frame_id != f"aruco_marker_{MARKER_ID}":
            return                                   # ignore other IDs
        try:
            base_pose = self.tf_buf.transform(msg, ROBOT_ROOT,
                                              timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception as e:
            self.get_logger().warn(f"TF failed: {e}")
            return
        base_pose.pose.position.z += OFFSET_Z       # apply 5 cm lift
        if self.moveit.go_to_pose(base_pose.pose):
            self.get_logger().info("Reached target.")
        else:
            self.get_logger().warn("Planning failed.")

def main():
    rclpy.init()
    rclpy.spin(MarkerFollower())

if __name__ == "__main__":
    main()
