#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.bridge = CvBridge()
        # add image_topic variable, change camera name 
        # image_sub = self.create_subscription(CompressedImage, 'camera/image', self.image_callback, 10)
        # get rid of image_transport

        image_transport = ImageTransport(
            node=self, name='imagetransport_sub', image_transport='compressed'
        )
        image_transport.subscribe('camera/image', 10, self.image_callback)

        self.twist_pub = self.create_publisher(Twist, 'servo_cmd_vel', 10)

        self.cv_image = None

        # Timer (30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info('Image Subscriber node initialized.')

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        if self.cv_image.shape[1] > 60 and self.cv_image.shape[0] > 60:
            cv2.circle(self.cv_image, (50, 50), 10, (0, 255, 0), -1)

        # ArUco marker detection code commented out
        """
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(self.cv_image, aruco_dict, parameters=parameters)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(self.cv_image, corners, ids)
            marker_center = np.mean(corners[0].reshape((4, 2)), axis=0)
            image_center = np.array([cols / 2, rows / 2])
            displacement = marker_center - image_center
            cv2.arrowedLine(self.cv_image,
                            (int(image_center[0]), int(image_center[1])),
                            (int(marker_center[0]), int(marker_center[1])),
                            (255, 0, 0), 2)
            twist = Twist()
            scale = 0.001
            twist.linear.x = displacement[1] * scale
            twist.linear.y = displacement[0] * scale
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)
            self.get_logger().info('Displacement vector: {}'.format(displacement))
        """

    def timer_callback(self):
        if self.cv_image is not None:
            cv2.imshow('Processed Image', self.cv_image)
            cv2.waitKey(1)  

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

