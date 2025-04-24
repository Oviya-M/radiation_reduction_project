#!/usr/bin/env python3

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2




class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')
        
        self.bridge = CvBridge()


        self.publisher_ = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        marker_id = 42
        marker_size = 200
        marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
        
        # Create a dummy image (480x640 with 3 channels)
        img = np.uint8(np.random.randint(0, 255, (480, 640, 3)))

        # Convert marker to BGR (from grayscale)
        marker_bgr = cv2.cvtColor(marker_image, cv2.COLOR_GRAY2BGR)

        # Place the marker on the image at a specific location
        x_offset = 220
        y_offset = 140
        img[y_offset:y_offset+marker_size, x_offset:x_offset+marker_size] = marker_bgr

        
        # Encode as JPEG and create CompressedImage message
        success, buffer = cv2.imencode('.jpg', img) # encodes an image to jpeg format
        if not success:
            self.get_logger().error('Failed to encode image')
            return

        msg = CompressedImage() # creates ros2 message of type CompressedImage
        msg.header.stamp = self.get_clock().now().to_msg() # adds timestamp to msg
        msg.header.frame_id = 'camera' # sets the frame id
        msg.format = "jpeg" # compression format of the image
        msg.data = buffer.tobytes() # convert numpy buffer to raw byte

        self.publisher_.publish(msg)
        self.get_logger().info('Published compressed image')
        

def main(args=None):
    rclpy.init(args=args)
    node = TestImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()