#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

import time
import numpy as np
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError

import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header, Float64, Float64MultiArray
from sensor_msgs.msg import PointCloud2, Image, JointState

import tf2_ros
from tf2_geometry_msgs import PointStamped

import threading

from binocular_laser_profile_scanner.thinning import find_center_pixels
from binocular_laser_profile_scanner.laser_triangulation import calculate_multiple_pos

class BinocularScanner():
    def __init__(self):
        # ----------------------------------------------------------
        # INIT VARIABLES
        # ----------------------------------------------------------
        self.right_cam_img = None
        self.left_cam_img = None
        self.show_img = False

        # Rot table scan angles
        from_angle = 0.0
        to_angle = math.pi * 2
        increment = math.pi / 180
        self.rot_table_scan_angles = np.arange(from_angle, to_angle, increment)

        self.rot_table_pos_msg = 0
        self.scanner_pos_msg = 0
        
        self.scanner_positions = Float64MultiArray()
        
        self.initial_rot_table_pos_cmd = 0 * math.pi / 180
        self.initial_scanner_pos_cmd = 15 * math.pi / 180

        self.points = []
        self.transformed_points = []
        self.transformed_point_cloud = None
        self.point_cloud_frame_id = "line_laser_link"
        self.point_cloud_source_frame = "line_laser_link"
        # self.point_cloud_target_frame = "rot_table_link"
        self.point_cloud_target_frame = "rot_table_link"

        # ----------------------------------------------------------
        # INIT TOPICS
        # ----------------------------------------------------------
        self.left_cam_topic = "left_camera/image_raw"
        self.right_cam_topic = "right_camera/image_raw"
        self.joint_states_topic = "joint_states"

        self.scanner_positions_cmd_topic = "position_controller/commands"

        # ----------------------------------------------------------
        # INIT ROS NODE
        # ----------------------------------------------------------
        print("Init node started...")
        self.scanner_node = rclpy.create_node('scanner_node')
        print("Init node done...")

        # ----------------------------------------------------------
        # INIT SUBSCRIBERS
        # ----------------------------------------------------------
        self.left_cam_image_subcriber = self.scanner_node.create_subscription(Image, self.left_cam_topic, self.callback_left_cam, 1)
        print("Subscribed images from topic /" + self.left_cam_topic)

        self.right_cam_image_subcriber = self.scanner_node.create_subscription(Image, self.right_cam_topic, self.callback_right_cam, 1)
        print("Subscribed images from topic /" + self.right_cam_topic)

        self.joint_state_subcriber = self.scanner_node.create_subscription(JointState, self.joint_states_topic, self.callback_joint_states, 1)
        print("Subscribed joint states from topic /" + self.joint_states_topic)

        # ----------------------------------------------------------
        # INIT PUBLISHERS
        # ----------------------------------------------------------
        self.point_cloud_pub = self.scanner_node.create_publisher(PointCloud2, 'point_cloud', 5)
        self.scan_rate = self.scanner_node.create_rate(5) #Hz

        self.scanner_pos_pub = self.scanner_node.create_publisher(Float64MultiArray, self.scanner_positions_cmd_topic, 1)

        if self.show_img is True:
            # cv2.imshow is not thread safe. Thats why, I create 2 named window to display camera images
            cv2.namedWindow("left_img", cv2.WINDOW_NORMAL)
            cv2.namedWindow("right_img", cv2.WINDOW_NORMAL)
        
        rclpy.spin_once(self.scanner_node)
        print("init done")

        # ----------------------------------------------------------
        # INIT THREADS
        # ----------------------------------------------------------
        self.spin_node_thread = threading.Thread(target=rclpy.spin, args=(self.scanner_node, ), daemon=True)
        self.spin_node_thread.start()

        self.msg_lock = threading.Lock()
    
    def run(self):
        # ----------------------------------------------------------
        # RESET SCANNER AND ROT TABLE ANGLE
        # ----------------------------------------------------------
        while rclpy.ok():
            self.scanner_positions.data = [self.initial_rot_table_pos_cmd,
                                           self.initial_scanner_pos_cmd,
                                           0.0]
            
            self.scanner_pos_pub.publish(self.scanner_positions)
            
            print(f"Rot table position:\n"
                  f"(command) {self.initial_rot_table_pos_cmd}\n"
                  f"(current) {self.rot_table_pos_msg}\n\n"
                  
                  f"Scanner angular position:\n"
                  f"(command) {self.initial_scanner_pos_cmd}\n"
                  f"(current) {self.scanner_pos_msg}\n"
                  f"---------------------------------------------")

            
            is_rot_table_reset = 0.02 > abs(self.initial_rot_table_pos_cmd - self.rot_table_pos_msg)
            is_scanner_angle_reset = 0.02 > abs(self.initial_scanner_pos_cmd - self.scanner_pos_msg)
            
            if is_rot_table_reset and is_scanner_angle_reset:
                print("Scanner arrived initial positions.")
                break

            self.scan_rate.sleep()

        # ----------------------------------------------------------
        # START SCANNING
        # ----------------------------------------------------------
        time.sleep(2)
        print("Scan Started!\n")

        for rot_table_angle in self.rot_table_scan_angles:
            while rclpy.ok():
                
                self.scanner_positions.data = [rot_table_angle,
                                               self.initial_scanner_pos_cmd,
                                               0.0]
                
                self.scanner_pos_pub.publish(self.scanner_positions)

                is_rot_table_arrive = 0.02 > abs(rot_table_angle - self.rot_table_pos_msg)

                if is_rot_table_arrive:

                    if self.right_cam_img is not None:
                        print("Scan started at angle: ", rot_table_angle*180/math.pi)
                        self.scan_img(self.left_cam_img, self.right_cam_img)
                        self.publish_point_cloud(self.points)
                        break
                    else:
                        print("File write error")
                
                self.scan_rate.sleep()

            print("Scanned completed at angle: ", rot_table_angle*180/math.pi)
            print()
        
        print("Scan completed")
        self.save_point_cloud(self.transformed_point_cloud,
                              "/home/ali/ros2_ws/src/binocular_laser_profile_scanner/scans/scanned_point_cloud")

        # ----------------------------------------------------------
        # PUBLISH POINT CLOUD UNTIL SHUTDOWN
        # ----------------------------------------------------------
        while rclpy.ok():
            self.point_cloud_pub.publish(self.transformed_point_cloud)
            self.scan_rate.sleep()
    
    # ----------------------------------------------------------
    # MESSAGE CALLBACKS
    # ----------------------------------------------------------
    def callback_left_cam(self, image_msg):
        try:
            bridge = CvBridge()
            self.left_cam_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as error:
            self.scanner_node.get_logger().error(error)

        if self.show_img is True:
            cv2.imshow("left_img", self.left_cam_img)
            cv2.waitKey(1)
    
    def callback_right_cam(self, image_msg):
        try:
            bridge = CvBridge()
            self.right_cam_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as error:
            self.scanner_node.get_logger().error(error)

        if self.show_img is True:
            cv2.imshow("right_img", self.right_cam_img)
            cv2.waitKey(1)

    def callback_joint_states(self, joint_state_msg):
        self.rot_table_pos_msg = joint_state_msg.position[0]
        self.scanner_pos_msg = joint_state_msg.position[1]
    
    # ----------------------------------------------------------
    # SCAN IMAGES FUNCTION
    # ----------------------------------------------------------
    def scan_img(self, left_cam_img, right_cam_img):
        # Laser triangulation formula calculated according to right cam image.
        # Since right and left cam images are symmetrical, 
        # you can just flip left cam image vertically and use same laser triangulation formula.

        # thinning
        thinned_img_pixels= find_center_pixels(left_cam_img, right_cam_img)

        # laser triangulation
        if thinned_img_pixels is not None:
            self.points = calculate_multiple_pos(thinned_img_pixels)
    
    # ----------------------------------------------------------
    # CREATE AND PUBLISH POINT CLOUD FUNCTION
    # ----------------------------------------------------------
    def publish_point_cloud(self, points):
        # creating point cloud
        header = Header()
        header.stamp = self.scanner_node.get_clock().now().to_msg()
        header.frame_id = self.point_cloud_frame_id
        
        point_cloud = pc2.create_cloud_xyz32(header, points)

        # transforming point cloud from laser frame to rot table frame
        self.transformed_point_cloud = self.transform_point_cloud(point_cloud,
                                                          from_frame=self.point_cloud_source_frame,
                                                          to_frame=self.point_cloud_target_frame)

        # publishing transformed point cloud
        self.point_cloud_pub.publish(self.transformed_point_cloud)
    
    # ----------------------------------------------------------
    # TRANSFORM POINT CLOUD
    # ----------------------------------------------------------    
    def transform_point_cloud(self, point_cloud, from_frame, to_frame):
        try:
            # getting transformation matrix from /tf2 topic
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer, self.scanner_node)
            transform = tf_buffer.lookup_transform(target_frame=to_frame,
                                                   source_frame=from_frame,
                                                   time=Time(),
                                                   timeout=Duration(seconds=1))
            
            # transforming every point to rot table frame
            for point in pc2.read_points(point_cloud, skip_nans=True):
                p = PointStamped()
                p.header.frame_id = from_frame
                p.point.x = float(point[0])
                p.point.y = float(point[1])
                p.point.z = float(point[2])
                transformed_point = tf_buffer.transform(p, to_frame, Duration(seconds=1))
                self.transformed_points.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
            
            # creating point cloud with transformed points
            point_cloud.header.frame_id = to_frame
            new_cloud = pc2.create_cloud_xyz32(point_cloud.header, np.array(self.transformed_points))
            return new_cloud
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.scanner_node.get_logger().error("Cant obtain transformation matrix!")
        
        return None
    
    # ----------------------------------------------------------
    # SAVE POINT CLOUD FUNCTION
    # ----------------------------------------------------------
    def save_point_cloud(self, point_cloud, point_cloud_name):
        try:
            with open(point_cloud_name + ".pcd", "w") as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write("WIDTH {}\n".format(point_cloud.width))
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write("POINTS {}\n".format(point_cloud.width))
                f.write("DATA ascii\n")
                
                for point in pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True):
                    f.write("{} {} {}\n".format(point[0], point[1], point[2]))
                    
            print("Point cloud saved: " + point_cloud_name + ".pcd")

        except IOError:
            self.scanner_node.get_logger().error("File write error")
    



def main(args=None):
    rclpy.init(args=args)
    print("rclpy init done")
    
    scanner = BinocularScanner()
    scanner.run()

    rclpy.shutdown()
    scanner.spin_node_thread.join()

if __name__ == '__main__':
    main()

