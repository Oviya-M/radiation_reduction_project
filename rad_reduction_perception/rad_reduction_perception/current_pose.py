#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
from transformations import euler_from_quaternion
import transformations as tf_transformations

class CurrentPosePublisher(Node):
    def __init__(self):
        super().__init__('current_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/current_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('g_base', 'joint6_flange', now)
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation
            self.publisher_.publish(pose)
            self.get_logger().info('Published current pose.')
        except Exception as e:
            self.get_logger().warn(f'Could not transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CurrentPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from pymoveit2 import MoveIt2
# from pymoveit2.robots import mycobot280 as robot
# from geometry_msgs.msg import PoseStamped

# class CurrentPosePublisher(Node):
#     def __init__(self):
#         super().__init__('current_pose_publisher')

#         self.moveit2 = MoveIt2(
#             node=self,
#             joint_names=robot.joint_names(),
#             base_link_name=robot.base_link_name(),
#             end_effector_name=robot.end_effector_name(),
#             group_name=robot.MOVE_GROUP_ARM,
#         )

#         self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

#         self.timer = self.create_timer(1.0, self.publish_pose)

#     def publish_pose(self):
#         pose = self.moveit2.current_pose()
#         self.pose_pub.publish(pose)
#         self.get_logger().info(f"Published pose: {pose}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CurrentPosePublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
