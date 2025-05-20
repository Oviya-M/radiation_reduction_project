#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
# from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from transformations import euler_from_quaternion
import transformations as tf_transformations
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
import yaml

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.bridge = CvBridge()
        # add image_topic variable, change camera name 
        # image_sub = self.create_subscription(CompressedImage, 'camera/image', self.image_callback, 10)
        # get rid of image_transport

        self.subscription = self.create_subscription(
            CompressedImage, '/image_rect/compressed', self.image_callback, 10
        )

        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.aruco_pose_pub = self.create_publisher(PoseStamped, 'aruco_poses', 10)

        self.cv_image = None

        # Timer (30Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info('Image Subscriber node initialized.')

    def image_callback(self, msg):
        try:
            # self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return


        # ArUco marker detection code 
        if self.cv_image is not None and self.cv_image.shape[1] > 60 and self.cv_image.shape[0] > 60:
            # Detect ArUco marker

             # Convert the image to grayscale
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

             # Define the dictionary and parameters
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            parameters = cv2.aruco.DetectorParameters_create()

             # Create the ArUco detector + detect the markers
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters) 



   # Estimate pose of aruco marker using corners and real world marker length
             # Import calibration file
            with open('/home/macs/.ros/camera_info/default_cam.yaml', 'r') as f:
                calib = yaml.safe_load(f)

             # Extract camera matrix
            mtx = np.array(calib['camera_matrix']['data']).reshape((3, 3))

             # Extract distortion coefficients
            distortion = np.array(calib['distortion_coefficients']['data'])

            marker_size = 0.1
            marker_points = np.array([
                    [-marker_size / 2, marker_size / 2, 0],
                    [marker_size / 2, marker_size / 2, 0],
                    [marker_size / 2, -marker_size / 2, 0],
                    [-marker_size / 2, -marker_size / 2, 0]
            ], dtype=np.float32)
            
            rvecs = [] # rotational vector (between marker and camera)
            tvecs = [] # translational vector (between marker and camera)
            trash = [] # unnecessary stuff??

             # for each of the markers detected, find pose
            for i, c in enumerate(corners):
                # SolvePnP gives rvec (Rodrigues vector) and tvec
                success, rvec, tvec = cv2.solvePnP(marker_points, c, mtx, distortion)
                tvecs.append(tvec)
                rvecs.append(rvec)
                if not success:
                    continue

                # Convert rvec to rotation matrix
                R, _ = cv2.Rodrigues(rvec)

                # Create 4x4 homogeneous transform matrix
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = tvec.flatten()

                # Optional: Apply 90° correction about x-axis if needed
                theta = np.pi / 2  # or -np.pi / 2 depending on your setup
                Rx = np.array([
                    [1, 0, 0, 0],
                    [0, np.cos(theta), np.sin(theta), 0],
                    [0, -np.sin(theta), np.cos(theta), 0],
                    [0, 0, 0, 1]
                ])

                # print(T)
                # T = Rx @ T  # apply correction in base frame
                # print(T)

                # Extract corrected translation and rotation
                t_corrected = T[:3, 3]
                R_corrected = T[:3, :3]
                quat = tf_transformations.quaternion_from_matrix(T)

                # --- PoseStamped ---
                aruco_pose_stamped = PoseStamped()
                aruco_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                aruco_pose_stamped.header.frame_id = 'joint6_flange'  # your camera's frame

                aruco_pose_stamped.pose.position.x = float(t_corrected[0])
                aruco_pose_stamped.pose.position.y = float(t_corrected[1])
                aruco_pose_stamped.pose.position.z = float(t_corrected[2])

                aruco_pose_stamped.pose.orientation.x = quat[0]
                aruco_pose_stamped.pose.orientation.y = quat[1]
                aruco_pose_stamped.pose.orientation.z = quat[2]
                aruco_pose_stamped.pose.orientation.w = quat[3]

                self.aruco_pose_pub.publish(aruco_pose_stamped)
                self.get_logger().info(f'Publishing pose for marker {ids[i][0]}')



            if ids is not None:
                cv2.aruco.drawDetectedMarkers(self.cv_image, corners, ids) # draws detected markers on the image by marking corners and displaying id
                print("HELLO WORLD")
                # Use the first detected marker
                marker_center = np.mean(corners[0][0], axis=0) # corner[0] is first marker and corner[0][0] is the 4 corner coordinates of the marker and mean takes the mean
                image_center = np.array([self.cv_image.shape[1] / 2, self.cv_image.shape[0] / 2]) # finds the center of the image
                
                displacement = marker_center - image_center

                # Draw direction vector
                cv2.arrowedLine(
                    self.cv_image,
                    tuple(image_center.astype(int)),
                    tuple(marker_center.astype(int)),
                    (255, 0, 0),
                    2
                )

                # Create and publish Twist message
                twist_stamped = TwistStamped()
                scale = 0.002  # Tune this value
                twist_stamped.header.stamp = self.get_clock().now().to_msg()   # ← add this
                twist_stamped.header.frame_id = 'joint6_flange'
                twist_stamped.twist.linear.x = displacement[1] * scale  # Vertical movement
                twist_stamped.twist.linear.y = displacement[0] * scale  # Horizontal movement
                self.twist_pub.publish(twist_stamped)

                self.get_logger().info(f'Displacement vector: {displacement}')

                for i in range(len(rvecs)):
                    self.get_logger().info(f"[Pose] Marker {ids[i][0]}: rvec = {rvecs[i].ravel()}, tvec = {tvecs[i].ravel()}")
                    cv2.aruco.drawAxis(self.cv_image, mtx, distortion, rvecs[i], tvecs[i], marker_size * 0.5)

            else:
                self.get_logger().info('No ArUco marker detected.')
        

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

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# # from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np
# from transformations import euler_from_quaternion
# import transformations as tf_transformations
# from geometry_msgs.msg import Twist, PoseStamped
# import yaml

# class MySubscriber(Node):
#     def __init__(self):
#         super().__init__('my_subscriber')
#         self.bridge = CvBridge()
        
#         self.subscription = self.create_subscription(
#             CompressedImage, '/image_raw/compressed', self.image_callback, 10,
#             qos_profile=rclpy.qos.qos_profile_sensor_data
#         )

#         self.twist_pub = self.create_publisher(Twist, 'servo_cmd_vel', 10)

#         self.pose_pub = self.create_publisher(PoseStamped, '/aruco/poses', 10)

#         self.cv_image = None

#         # Timer (30Hz)
#         self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

#         self.get_logger().info('Image Subscriber node initialized.')

#     def image_callback(self, msg):
#         try:
#             # self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             np_arr = np.frombuffer(msg.data, np.uint8)
#             self.cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
#         except CvBridgeError as e:
#             self.get_logger().error(f'CvBridge Error: {e}')
#             return


#         # ArUco marker detection code 
#         if self.cv_image is not None and self.cv_image.shape[1] > 60 and self.cv_image.shape[0] > 60:
#             # Detect ArUco marker

#              # Convert the image to grayscale
#             gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

#              # Define the dictionary and parameters
#             aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#             parameters = cv2.aruco.DetectorParameters_create()

#              # Create the ArUco detector + detect the markers
#             corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters) 



#             # Estimate pose of aruco marker using corners and real world marker length
#              # Import calibration file
#             with open('/home/macs/.ros/camera_info/default_cam.yaml', 'r') as f:
#                 calib = yaml.safe_load(f)

#              # Extract camera matrix
#             mtx = np.array(calib['camera_matrix']['data']).reshape((3, 3))

#              # Extract distortion coefficients
#             distortion = np.array(calib['distortion_coefficients']['data'])

#             marker_size = 0.1
#             marker_points = np.array([
#                     [-marker_size / 2, marker_size / 2, 0],
#                     [marker_size / 2, marker_size / 2, 0],
#                     [marker_size / 2, -marker_size / 2, 0],
#                     [-marker_size / 2, -marker_size / 2, 0]
#             ], dtype=np.float32)
            
#             rvecs = [] # rotational vector (between marker and camera)
#             tvecs = [] # translational vector (between marker and camera)
#             trash = [] # unnecessary stuff??

#              # for each of the markers detected, find pose
#             for i, c in enumerate(corners):
#                 nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion)

#                 # we need a homogeneous matrix but OpenCV only gives us a 3x3 rotation matrix
#                 rotation_matrix = np.array([[0, 0, 0, 0],
#                                             [0, 0, 0, 0],
#                                             [0, 0, 0, 0],
#                                             [0, 0, 0, 1]],
#                                             dtype=float)
#                 rotation_matrix[:3, :3], _ = cv2.Rodrigues(R)

#                 # Put t into rotation_matrix to make it a homogeneous transform matrix
#                 rotation_matrix[0,3] = t[0]
#                 rotation_matrix[1,3] = t[1]
#                 rotation_matrix[2,3] = t[2]
#                 print(rotation_matrix)

#                 # Apply 90 degree rotation about x-axis to 
#                 theta = np.pi / 2
#                 # theta = np.pi

#                 # Rotation matrix about X axis
#                 Rx = np.array([
#                     [1, 0,           0,          0],
#                     [0, np.cos(theta), np.sin(theta), 0],
#                     [0, -np.sin(theta),  np.cos(theta), 0],
#                     [0, 0,           0,          1]
#                 ])

#                 rotation_matrix = np.dot(Rx, rotation_matrix)

#                 print(rotation_matrix)

#                 # Unpack t from rotation_matrix
#                 R = rotation_matrix[:3,:3]
#                 t = rotation_matrix[:3, 3]
#                 rvecs.append(R)
#                 tvecs.append(t)

#                 # convert the matrix to a quaternion
#                 quaternion = tf_transformations.quaternion_from_matrix(rotation_matrix)


#                 # Create a PoseStamped ROS2 message: https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html
#                 aruco_pose_stamped = PoseStamped()
#                 marker_id = int(ids[i][0])
#                 aruco_pose_stamped.header.frame_id = f'aruco_marker_{marker_id}'
#                 self.pose_pub.publish(aruco_pose_stamped)

#               # aruco_pose_stamped.header.stamp = self.get_clock().now().to_msg()
#               # aruco_pose_stamped.header.frame_id = 'camera_flange'

#               # aruco_pose_stamped.pose.position.x = float(tvecs[i][0])
#               # aruco_pose_stamped.pose.position.y = float(tvecs[i][1])
#               # aruco_pose_stamped.pose.position.z = float(tvecs[i][2])

#               # aruco_pose_stamped.pose.orientation.x = quaternion[0]
#               # aruco_pose_stamped.pose.orientation.y = quaternion[1]
#               # aruco_pose_stamped.pose.orientation.z = quaternion[2]
#               # aruco_pose_stamped.pose.orientation.w = quaternion[3]
#               #self.aruco_pose_pub.publish(aruco_pose_stamped)




#             if ids is not None:
#                 cv2.aruco.drawDetectedMarkers(self.cv_image, corners, ids) # draws detected markers on the image by marking corners and displaying id

#                 # Use the first detected marker
#                 marker_center = np.mean(corners[0][0], axis=0) # corner[0] is first marker and corner[0][0] is the 4 corner coordinates of the marker and mean takes the mean
#                 image_center = np.array([self.cv_image.shape[1] / 2, self.cv_image.shape[0] / 2]) # finds the center of the image
                
#                 displacement = marker_center - image_center

#                 # Draw direction vector
#                 cv2.arrowedLine(
#                     self.cv_image,
#                     tuple(image_center.astype(int)),
#                     tuple(marker_center.astype(int)),
#                     (255, 0, 0),
#                     2
#                 )

#                 # Create and publish Twist message
#               # twist = Twist()
#               # scale = 0.002  # Tune this value
#               # twist.linear.x = displacement[1] * scale  # Vertical movement
#               # twist.linear.y = displacement[0] * scale  # Horizontal movement
#               # self.twist_pub.publish(twist)

#                 self.get_logger().info(f'Displacement vector: {displacement}')
#                 # test_rvec = np.array([[0.0], [0.0], [0.0]], dtype=np.float32)
#                 # test_tvec = np.array([[0.0], [0.0], [0.5]], dtype=np.float32)
#                 # cv2.aruco.drawAxis(self.cv_image, mtx, distortion, test_rvec, test_tvec, 0.1)

#                 for i in range(len(rvecs)):
#                     rvec, _ = cv2.Rodrigues(rvecs[i])  # Convert 3x3 matrix back to 3x1 vector
#                     self.get_logger().info(f"[Pose] Marker {ids[i][0]}: rvec = {rvec.ravel()}, tvec = {tvecs[i].ravel()}")
#                     cv2.aruco.drawAxis(self.cv_image, mtx, distortion, rvec, tvecs[i], marker_size * 0.5)

#             else:
#                 self.get_logger().info('No ArUco marker detected.')
        

#     def timer_callback(self):
#         if self.cv_image is not None:
#             cv2.imshow('Processed Image', self.cv_image)
#             cv2.waitKey(1)  

# def main(args=None):
#     rclpy.init(args=args)
#     node = MySubscriber()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info('Keyboard Interrupt, shutting down.')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()



