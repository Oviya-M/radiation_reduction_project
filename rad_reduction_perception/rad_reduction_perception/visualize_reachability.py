import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2
from pymoveit2.robots import mycobot280 as robot

class ReachabilityVisualizer(Node):
    def __init__(self):
        super().__init__('visualize_reachability')

        self.marker_pub = self.create_publisher(Marker, '/reachability_markers', 10)

        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5

        self.timer = self.create_timer(0.5, self.test_reachability)
        self.grid_points = self.generate_grid([-0.2, 0.2], [-0.2, 0.2], [0.1, 0.3], 0.05)
        self.index = 0

        self.get_logger().info("Reachability visualizer initialized")

    def generate_grid(self, x_range, y_range, z_range, step):
        x_vals = np.arange(x_range[0], x_range[1], step)
        y_vals = np.arange(y_range[0], y_range[1], step)
        z_vals = np.arange(z_range[0], z_range[1], step)
        return [(x, y, z) for x in x_vals for y in y_vals for z in z_vals]

    def test_reachability(self):
        if self.index >= len(self.grid_points):
            self.get_logger().info("Done testing reachability.")
            return

        x, y, z = self.grid_points[self.index]
        self.index += 1

        quat = [0.0, 0.0, 0.0, 1.0]  # identity quaternion

        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'reachability'
        marker.id = self.index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.02

        future = self.moveit2.move_to_pose(
            position=[x, y, z],
            quat_xyzw=quat,
            cartesian=False,
            cartesian_max_step=0.0025,
            cartesian_fraction_threshold=0.0,
        )

        def callback(fut):
            if fut.result():
                marker.color.r = 0.0  # green
                marker.color.g = 1.0
            else:
                marker.color.r = 1.0  # red
                marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            self.marker_pub.publish(marker)

        if future is not None:
            future.add_done_callback(callback)
        else:
            self.get_logger().warn("Motion planning failed — no future returned.")

def main(args=None):
    rclpy.init(args=args)
    node = ReachabilityVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
