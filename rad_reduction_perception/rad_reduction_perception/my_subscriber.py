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
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, PointStamped
import yaml

from pytransform3d.rotations import matrix_from_quaternion
from pymoveit2 import MoveIt2
from pymoveit2.robots import mycobot280 as robot
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.bridge = CvBridge()
        # add image_topic variable, change camera name 
        # image_sub = self.create_subscription(CompressedImage, 'camera/image', self.image_callback, 10)
        # get rid of image_transport




        # Add callback group for MoveIt2
        callback_group = ReentrantCallbackGroup()

        # Initialize MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        # Optional tuning
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5

        # TF2 transform lookup allows us to find current pose of robot end effector
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.subscription = self.create_subscription(
            CompressedImage, '/image_rect/compressed', self.image_callback, 10
        )

        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

        self.goal_position = None
        self.goal_orientation = None

        self.aruco_pose_pub = self.create_publisher(PoseStamped, 'aruco_poses', 10)

        # test pose
        self.test_pose_pub = self.create_publisher(PoseStamped, '/test_pose', 10)

        self.goal_point_pub = self.create_publisher(PointStamped, '/goal_point', 10)

        self.cv_image = None

        # Timer (30Hz)
        self.timer = self.create_timer(1.0 / 5.0, self.timer_callback)
        self.get_logger().info("Path planning timer initialized")

        self.path_planning_timer = self.create_timer(1/5.0, self.path_planning_callback)

        self.get_logger().info('Image Subscriber node initialized.')


        # debugging stuff
        # import threading
        # self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        # self.display_thread.start()


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
                # Get transform from joint6_flange to g_base
                try:
                    tf = self.tf_buffer.lookup_transform(
                        'g_base',
                        'joint6_flange',
                        rclpy.time.Time())

                    T_j6_gb = np.eye(4)
                    T_j6_gb[0,3] = tf.transform.translation.x
                    T_j6_gb[1,3] = tf.transform.translation.y
                    T_j6_gb[2,3] = tf.transform.translation.z

                    tf_quat = [tf.transform.rotation.w,
                            tf.transform.rotation.x,
                            tf.transform.rotation.y,
                            tf.transform.rotation.z]

                    R_j6_gb = matrix_from_quaternion(tf_quat)
                    T_j6_gb[:3, :3] = R_j6_gb

                    # Apply transform to ArUco pose
                    T_j6_a = T  # from cv2.solvePnP
                    T_gb_a = T_j6_gb @ T_j6_a

                    quat = tf_transformations.quaternion_from_matrix(T_gb_a)
                    pos = T_gb_a[:3, 3]

                    aruco_pose_stamped.header.frame_id = 'g_base'
                    aruco_pose_stamped.pose.position.x = float(pos[0])
                    aruco_pose_stamped.pose.position.y = float(pos[1])
                    aruco_pose_stamped.pose.position.z = float(pos[2])
                    aruco_pose_stamped.pose.orientation.x = quat[0]
                    aruco_pose_stamped.pose.orientation.y = quat[1]
                    aruco_pose_stamped.pose.orientation.z = quat[2]
                    aruco_pose_stamped.pose.orientation.w = quat[3]

                except TransformException as ex:
                    self.get_logger().error(f'Could not lookup transform g_base -> joint6_flange: {ex}')
                    return


                self.aruco_pose_pub.publish(aruco_pose_stamped)
                self.get_logger().info(f'Publishing pose for marker {ids[i][0]}')



            if ids is not None:
                # self.goal_position = [0.2, 0.0, 0.15]  # or from ArUco detection
                # self.goal_orientation = [0.0, 0.0, 0.0, 1.0]

                # for aruco pose
                # self.goal_position = t_corrected.tolist()
                # self.goal_orientation = quat.tolist()

                T_j6_a = T

                # # Adjust z_off to suit demo
                # z_off = 0.5
                # goal_position = [0., 0., z_off]
                # goal_z_hat = [0., 0., -1.]
                # goal_x_hat = [-1., 0., 0.]
                # goal_y_hat = np.cross(goal_z_hat, goal_x_hat)

                # # Normalize axes
                # goal_x_hat /= np.linalg.norm(goal_x_hat)
                # goal_y_hat /= np.linalg.norm(goal_y_hat)
                # goal_z_hat /= np.linalg.norm(goal_z_hat)

                # T_a_g = np.eye(4)
                # T_a_g[0:3,3] = goal_position
                # # T_a_g[0:3,0] = goal_x_hat
                # # T_a_g[0:3,1] = goal_y_hat
                # # T_a_g[0:3,2] = goal_z_hat


                # 1. Extract rotation from ArUco in g_base
                R_gb_a = T_gb_a[:3, :3]
                p_gb_a = T_gb_a[:3, 3]

                # 2. ArUco axes in g_base
                aruco_z = R_gb_a[:, 2]
                aruco_x = R_gb_a[:, 0]

                # 3. Flip Z-axis to point opposite direction
                goal_z = -aruco_z
                goal_x = -aruco_x  # optional: keep tool facing marker
                goal_y = np.cross(goal_z, goal_x)

                # Normalize
                goal_x /= np.linalg.norm(goal_x)
                goal_y /= np.linalg.norm(goal_y)
                goal_z /= np.linalg.norm(goal_z)

                # 4. Build new goal pose in g_base with 50 cm offset
                offset = -0.5
                T_gb_g = np.eye(4)
                T_gb_g[:3, 0] = goal_x
                T_gb_g[:3, 1] = goal_y
                T_gb_g[:3, 2] = goal_z
                T_gb_g[:3, 3] = p_gb_a + goal_z * offset  # offset along flipped Z

                # 5. Extract position and orientation
                self.goal_position = T_gb_g[:3, 3].tolist()
                self.goal_orientation = tf_transformations.quaternion_from_matrix(T_gb_g).tolist()

                # Optional: log result
                self.get_logger().info(f"Final goal in g_base:\npos={self.goal_position}\nori={self.goal_orientation}")


        

                # Generate pose offset from aruco pose for goal
                

                cv2.aruco.drawDetectedMarkers(self.cv_image, corners, ids) # draws detected markers on the image by marking corners and displaying id
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
                self.goal_position = None
                self.goal_orientation = None
        

    # def timer_callback(self):
    #     if self.cv_image is not None:
    #         cv2.imshow('Processed Image', self.cv_image)
    #         cv2.waitKey(1)  
    def timer_callback(self):
        if self.cv_image is not None:
            try:
                cv2.imshow('Processed Image', self.cv_image)
                key = cv2.waitKey(1)
                pass
            except Exception as e:
                self.get_logger().warn(f"cv2 error: {e}")
    
    # def display_loop(self):
    #     while rclpy.ok():
    #         if self.cv_image is not None:
    #             try:
    #                 cv2.imshow("Processed Image", self.cv_image)
    #                 key = cv2.waitKey(1)
    #             except Exception as e:
    #                 self.get_logger().warn(f"cv2 error: {e}")



    def path_planning_callback(self):

        if self.goal_position is None or self.goal_orientation is None:
            self.get_logger().info("Skipping planning — goal not set.")
            return

        self.get_logger().info("Planning callback triggered!")  # <-- ADD THIS
        # Always move to a fixed pose, regardless of ArUco result
        # self.goal_position = [0.2, 0.0, 0.15] # x, y, z in meters
        # self.goal_orientation = [0.0, 0.0, 0.0, 1.0] # no rotation (quaternion identity)



        self.get_logger().info("Moving to fixed test pose")

        # self.moveit2.move_to_pose(
        #     position=self.goal_position,
        #     quat_xyzw=self.goal_orientation,
        #     cartesian=False,
        #     cartesian_max_step=0.0025,
        #     cartesian_fraction_threshold=0.0,
        # )
        
        # self.moveit2.wait_until_executed()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        # pose_msg.header.frame_id = 'joint6_flange'  # Or base_link if you're using it
        pose_msg.header.frame_id = 'g_base'  # Or base_link if you're using it

        pose_msg.pose.position.x = self.goal_position[0]
        pose_msg.pose.position.y = self.goal_position[1]
        pose_msg.pose.position.z = self.goal_position[2]

        pose_msg.pose.orientation.x = self.goal_orientation[0]
        pose_msg.pose.orientation.y = self.goal_orientation[1]
        pose_msg.pose.orientation.z = self.goal_orientation[2]
        pose_msg.pose.orientation.w = self.goal_orientation[3]


        self.test_pose_pub.publish(pose_msg)
        self.get_logger().info("Published test pose to /test_pose")

        self.get_logger().warning(f"Calling move_to_pose with:\nposition={self.goal_position}\norientation={self.goal_orientation}")

        future = self.moveit2.move_to_pose(
            position=self.goal_position,
            quat_xyzw=self.goal_orientation,
            cartesian=False,
            cartesian_max_step=0.0025,
            cartesian_fraction_threshold=0.0,
        )

        # You can optionally log the planning started
        self.get_logger().info("Sent pose goal to MoveIt2")
        try:
            result = future.result(timeout=10.0)
            self.get_logger().info(f'MoveIt result: {result}')
        except Exception as e:
            self.get_logger().error(f'MoveIt planning/execution failed: {e}')

        # Don't block – allow other callbacks (like image) to run


        self.goal_position = None # x, y, z in meters
        self.goal_orientation = None



def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
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
# from geometry_msgs.msg import Twist, TwistStamped, PoseStamped, PointStamped
# import yaml

# from pytransform3d.rotations import matrix_from_quaternion
# from pymoveit2 import MoveIt2
# from pymoveit2.robots import mycobot280 as robot
# from rclpy.callback_groups import ReentrantCallbackGroup

# from tf2_ros import TransformException
# from tf2_ros.buffer import Buffer
# from tf2_ros.transform_listener import TransformListener



# class MySubscriber(Node):
#     def __init__(self):
#         super().__init__('my_subscriber')
#         self.bridge = CvBridge()
#         # add image_topic variable, change camera name 
#         # image_sub = self.create_subscription(CompressedImage, 'camera/image', self.image_callback, 10)
#         # get rid of image_transport




#         # Add callback group for MoveIt2
#         callback_group = ReentrantCallbackGroup()

#         # Initialize MoveIt2 interface
#         self.moveit2 = MoveIt2(
#             node=self,
#             joint_names=robot.joint_names(),
#             base_link_name=robot.base_link_name(),
#             end_effector_name=robot.end_effector_name(),
#             group_name=robot.MOVE_GROUP_ARM,
#             callback_group=callback_group,
#         )
#         self.moveit2.planner_id = "RRTConnectkConfigDefault"

#         # Optional tuning
#         self.moveit2.max_velocity = 0.5
#         self.moveit2.max_acceleration = 0.5

#         # TF2 transform lookup allows us to find current pose of robot end effector
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)


#         self.subscription = self.create_subscription(
#             CompressedImage, '/image_rect/compressed', self.image_callback, 10
#         )

#         self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)

#         self.goal_position = None
#         self.goal_orientation = None

#         self.aruco_pose_pub = self.create_publisher(PoseStamped, 'aruco_poses', 10)

#         # test pose
#         self.test_pose_pub = self.create_publisher(PoseStamped, '/test_pose', 10)

#         self.goal_point_pub = self.create_publisher(PointStamped, '/goal_point', 10)

#         self.cv_image = None

#         # Timer (30Hz)
#         self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
#         self.get_logger().info("Path planning timer initialized")

#         self.path_planning_timer = self.create_timer(1/5.0, self.path_planning_callback)

#         self.get_logger().info('Image Subscriber node initialized.')


#         # debugging stuff
#         # import threading
#         # self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
#         # self.display_thread.start()


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



#    # Estimate pose of aruco marker using corners and real world marker length
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
#                 # SolvePnP gives rvec (Rodrigues vector) and tvec
#                 success, rvec, tvec = cv2.solvePnP(marker_points, c, mtx, distortion)
#                 tvecs.append(tvec)
#                 rvecs.append(rvec)
#                 if not success:
#                     continue

#                 # Convert rvec to rotation matrix
#                 R, _ = cv2.Rodrigues(rvec)

#                 # Create 4x4 homogeneous transform matrix
#                 T = np.eye(4)
#                 T[:3, :3] = R
#                 T[:3, 3] = tvec.flatten()

#                 # Optional: Apply 90° correction about x-axis if needed
#                 theta = np.pi / 2  # or -np.pi / 2 depending on your setup
#                 Rx = np.array([
#                     [1, 0, 0, 0],
#                     [0, np.cos(theta), np.sin(theta), 0],
#                     [0, -np.sin(theta), np.cos(theta), 0],
#                     [0, 0, 0, 1]
#                 ])

#                 # print(T)
#                 # T = Rx @ T  # apply correction in base frame
#                 # print(T)

#                 # Extract corrected translation and rotation
#                 t_corrected = T[:3, 3]
#                 R_corrected = T[:3, :3]
#                 quat = tf_transformations.quaternion_from_matrix(T)

#                 # --- PoseStamped ---
#                 aruco_pose_stamped = PoseStamped()
#                 aruco_pose_stamped.header.stamp = self.get_clock().now().to_msg()
#                 aruco_pose_stamped.header.frame_id = 'joint6_flange'  # your camera's frame

#                 aruco_pose_stamped.pose.position.x = float(t_corrected[0])
#                 aruco_pose_stamped.pose.position.y = float(t_corrected[1])
#                 aruco_pose_stamped.pose.position.z = float(t_corrected[2])

#                 aruco_pose_stamped.pose.orientation.x = quat[0]
#                 aruco_pose_stamped.pose.orientation.y = quat[1]
#                 aruco_pose_stamped.pose.orientation.z = quat[2]
#                 aruco_pose_stamped.pose.orientation.w = quat[3]

#                 self.aruco_pose_pub.publish(aruco_pose_stamped)
#                 self.get_logger().info(f'Publishing pose for marker {ids[i][0]}')



#             if ids is not None:
#                 # self.goal_position = [0.2, 0.0, 0.15]  # or from ArUco detection
#                 # self.goal_orientation = [0.0, 0.0, 0.0, 1.0]

#                 # for aruco pose
#                 # self.goal_position = t_corrected.tolist()
#                 # self.goal_orientation = quat.tolist()

#                 T_j6_a = T

#                 # # Adjust z_off to suit demo
#                 # z_off = 0.5
#                 # goal_position = [0., 0., z_off]
#                 # goal_z_hat = [0., 0., -1.]
#                 # goal_x_hat = [-1., 0., 0.]
#                 # goal_y_hat = np.cross(goal_z_hat, goal_x_hat)

#                 # # Normalize axes
#                 # goal_x_hat /= np.linalg.norm(goal_x_hat)
#                 # goal_y_hat /= np.linalg.norm(goal_y_hat)
#                 # goal_z_hat /= np.linalg.norm(goal_z_hat)

#                 # T_a_g = np.eye(4)
#                 # T_a_g[0:3,3] = goal_position
#                 # # T_a_g[0:3,0] = goal_x_hat
#                 # # T_a_g[0:3,1] = goal_y_hat
#                 # # T_a_g[0:3,2] = goal_z_hat

#                 # Invert z-axis of ArUco marker's orientation (camera frame)
#                # Invert z-axis of ArUco marker's orientation (camera frame)
#                 R_aruco = T[:3, :3]  # Rotation from ArUco
#                 aruco_z_axis = R_aruco[:, 2]  # Third column is Z-axis
#                 aruco_x_axis = R_aruco[:, 0]  # First column is X-axis

#                 goal_z_hat = -aruco_z_axis  # Opposite direction of ArUco z-axis
#                 goal_x_hat = -aruco_x_axis  # Optional: align tool orientation to face ArUco
#                 goal_y_hat = np.cross(goal_z_hat, goal_x_hat)

#                 # Normalize
#                 goal_x_hat /= np.linalg.norm(goal_x_hat)
#                 goal_y_hat /= np.linalg.norm(goal_y_hat)
#                 goal_z_hat /= np.linalg.norm(goal_z_hat)


#                 # Set offset 5cm toward robot (along flipped Z)
#                 offset = 0.05  # 5 cm
#                 offset_translation = -aruco_z_axis * offset  # shift toward robot
#                 goal_translation = T[:3, 3] + offset_translation  # apply offset to ArUco position

#                 # Build transform from ArUco to goal
#                 T_a_g = np.eye(4)
#                 T_a_g[0:3, 0] = goal_x_hat
#                 T_a_g[0:3, 1] = goal_y_hat
#                 T_a_g[0:3, 2] = goal_z_hat
#                 T_a_g[:3, 3] = goal_translation # offset 10 cm in ArUco frame




#                 # Want transform from joint6_flange to goal. We have T_a_g (Transform from aruco to goal) and T_j6_a (Transform from joint6_flange to aruco)
#                 # T_j6_g = T_j6_a.dot(T_a_g)
#                 T_j6_g = T_j6_a @ T_a_g
#                 # T_j6_g = T_a_g

#                 self.goal_position = T_j6_g[0:3,3].tolist()
#                 self.goal_orientation = tf_transformations.quaternion_from_matrix(T_j6_g).tolist() 


#                 try:
#                     tf = self.tf_buffer.lookup_transform(
#                     'g_base',
#                     'joint6_flange',
#                     rclpy.time.Time())
#                     self.get_logger().warning('--------------------------')
#                     print(T_j6_g)

#                     T_gb_j6 = np.eye(4)
#                     T_gb_j6[0,3] = tf.transform.translation.x
#                     T_gb_j6[1,3] = tf.transform.translation.y
#                     T_gb_j6[2,3] = tf.transform.translation.z
#                     tf_quat = [tf.transform.rotation.w,
#                                tf.transform.rotation.x,
#                                tf.transform.rotation.y,
#                                tf.transform.rotation.z]
#                     # Transform the quaternion into a rotation matrix
#                     R_gb_j6 = matrix_from_quaternion(tf_quat)
#                     T_gb_j6[:3,:3] = R_gb_j6
#                     T_gb_g = T_gb_j6.dot(T_j6_g)
#                     # self.goal_position = T_gb_g[:3,3].tolist()
#                     # self.goal_orientation = tf_transformations.quaternion_from_matrix(T_gb_g).tolist() 

#                     self.get_logger().warning('--------------------------')
#                 except TransformException as ex:
#                     self.get_logger().info(
#                         f'Could not find transform g_base to joint6_flange: {ex}')
#                     return

        

#                 # Generate pose offset from aruco pose for goal
                

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
#                 twist_stamped = TwistStamped()
#                 scale = 0.002  # Tune this value
#                 twist_stamped.header.stamp = self.get_clock().now().to_msg()   # ← add this
#                 twist_stamped.header.frame_id = 'joint6_flange'

#                 twist_stamped.twist.linear.x = displacement[1] * scale  # Vertical movement
#                 twist_stamped.twist.linear.y = displacement[0] * scale  # Horizontal movement
#                 self.twist_pub.publish(twist_stamped)

#                 self.get_logger().info(f'Displacement vector: {displacement}')

#                 for i in range(len(rvecs)):
#                     self.get_logger().info(f"[Pose] Marker {ids[i][0]}: rvec = {rvecs[i].ravel()}, tvec = {tvecs[i].ravel()}")
#                     cv2.aruco.drawAxis(self.cv_image, mtx, distortion, rvecs[i], tvecs[i], marker_size * 0.5)

#             else:
#                 self.get_logger().info('No ArUco marker detected.')
#                 self.goal_position = None
#                 self.goal_orientation = None
        

#     # def timer_callback(self):
#     #     if self.cv_image is not None:
#     #         cv2.imshow('Processed Image', self.cv_image)
#     #         cv2.waitKey(1)  
#     def timer_callback(self):
#         if self.cv_image is not None:
#             try:
#                 cv2.imshow('Processed Image', self.cv_image)
#                 key = cv2.waitKey(1)
#                 pass
#             except Exception as e:
#                 self.get_logger().warn(f"cv2 error: {e}")
    
#     # def display_loop(self):
#     #     while rclpy.ok():
#     #         if self.cv_image is not None:
#     #             try:
#     #                 cv2.imshow("Processed Image", self.cv_image)
#     #                 key = cv2.waitKey(1)
#     #             except Exception as e:
#     #                 self.get_logger().warn(f"cv2 error: {e}")



#     def path_planning_callback(self):

#         if self.goal_position is None or self.goal_orientation is None:
#             self.get_logger().info("Skipping planning — goal not set.")
#             return

#         self.get_logger().info("Planning callback triggered!")  # <-- ADD THIS
#         # Always move to a fixed pose, regardless of ArUco result
#         # self.goal_position = [0.2, 0.0, 0.15] # x, y, z in meters
#         # self.goal_orientation = [0.0, 0.0, 0.0, 1.0] # no rotation (quaternion identity)



#         self.get_logger().info("Moving to fixed test pose")

#         # self.moveit2.move_to_pose(
#         #     position=self.goal_position,
#         #     quat_xyzw=self.goal_orientation,
#         #     cartesian=False,
#         #     cartesian_max_step=0.0025,
#         #     cartesian_fraction_threshold=0.0,
#         # )
        
#         # self.moveit2.wait_until_executed()

#         pose_msg = PoseStamped()
#         pose_msg.header.stamp = self.get_clock().now().to_msg()
#         pose_msg.header.frame_id = 'joint6_flange'  # Or base_link if you're using it

#         pose_msg.pose.position.x = self.goal_position[0]
#         pose_msg.pose.position.y = self.goal_position[1]
#         pose_msg.pose.position.z = self.goal_position[2]

#         pose_msg.pose.orientation.x = self.goal_orientation[0]
#         pose_msg.pose.orientation.y = self.goal_orientation[1]
#         pose_msg.pose.orientation.z = self.goal_orientation[2]
#         pose_msg.pose.orientation.w = self.goal_orientation[3]


#         self.test_pose_pub.publish(pose_msg)
#         self.get_logger().info("Published test pose to /test_pose")

#         self.get_logger().warning(f"Calling move_to_pose with:\nposition={self.goal_position}\norientation={self.goal_orientation}")

#         future = self.moveit2.move_to_pose(
#             position=self.goal_position,
#             quat_xyzw=self.goal_orientation,
#             cartesian=False,
#             cartesian_max_step=0.0025,
#             cartesian_fraction_threshold=0.0,
#         )

#         # You can optionally log the planning started
#         self.get_logger().info("Sent pose goal to MoveIt2")
#         try:
#             result = future.result(timeout=10.0)
#             self.get_logger().info(f'MoveIt result: {result}')
#         except Exception as e:
#             self.get_logger().error(f'MoveIt planning/execution failed: {e}')

#         # Don't block – allow other callbacks (like image) to run


#         self.goal_position = None # x, y, z in meters
#         self.goal_orientation = None



# def main(args=None):
#     rclpy.init(args=args)
#     node = MySubscriber()
#     try:
#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.1)
#     except KeyboardInterrupt:
#         node.get_logger().info('Keyboard Interrupt, shutting down.')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()
#         cv2.destroyAllWindows()



# if __name__ == '__main__':
#     main()
