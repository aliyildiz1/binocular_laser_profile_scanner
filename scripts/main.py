#!/usr/bin/python3

import rospy
import numpy as np
import math
import cv2
import time
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import PointCloud2, Image, JointState

from geometry_msgs.msg import Pose
import tf2_ros
from tf2_geometry_msgs import PointStamped

from cv_bridge import CvBridge, CvBridgeError
from threading import Lock

from thinning import thin
from laser_triangulation import calculate_multiple_pos



class binocular_scanner():
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
        
        self.points = []
        self.transformed_points = []
        self.pcl_transformed = None
        self.pcl_frame_id = "line_laser_link"
        self.pcl_source_frame = "line_laser_link"
        self.pcl_target_frame = "rot_table_link"
        
        self.rot_table_pos = Float64()
        self.scanner_pos = Float64()
        
        self.rot_table_pos.data = 0 * math.pi / 180
        self.scanner_pos.data = 15 * math.pi / 180

        # ----------------------------------------------------------
        # INIT THREAD LOCK
        # ----------------------------------------------------------
        self.msg_lock = Lock()

        # ----------------------------------------------------------
        # INIT TOPICS
        # ----------------------------------------------------------
        self.left_cam_topic = "binocular_laser_profile_scanner/left_camera/image_raw"
        self.right_cam_topic = "binocular_laser_profile_scanner/right_camera/image_raw"
        self.joint_states_topic = "binocular_laser_profile_scanner/joint_states"

        self.rot_table_pos_cmd_topic = "binocular_laser_profile_scanner/rot_table_angular_position_controller/command"
        self.scanner_pos_cmd_topic = "binocular_laser_profile_scanner/scanner_angular_position_controller/command"

        # ----------------------------------------------------------
        # INIT ROS NODE
        # ----------------------------------------------------------
        rospy.init_node("scanner_node", anonymous=True)

        # ----------------------------------------------------------
        # INIT SUBSCRIBERS
        # ----------------------------------------------------------
        self.left_cam_image_subcriber = rospy.Subscriber(self.left_cam_topic, Image, self.callback_left_cam, queue_size = 1, buff_size=2**8)
        print("Subscribed images from topic /" + self.left_cam_topic)

        self.right_cam_image_subcriber = rospy.Subscriber(self.right_cam_topic, Image, self.callback_right_cam, queue_size = 1, buff_size=2**8)
        print("Subscribed images from topic /" + self.right_cam_topic)

        self.joint_state_subcriber = rospy.Subscriber(self.joint_states_topic, JointState, self.callback_joint_states)
        print("Subscribed joint states from topic /" + self.joint_states_topic)

        # ----------------------------------------------------------
        # INIT PUBLISHERS
        # ----------------------------------------------------------
        self.pcl_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=1)
        self.pcl_pub_rate = rospy.Rate(5)  # 5 Hz

        self.rot_table_pos_pub = rospy.Publisher(self.rot_table_pos_cmd_topic, Float64, queue_size=1)
        self.scanner_pos_pub = rospy.Publisher(self.scanner_pos_cmd_topic, Float64, queue_size=1)

        if self.show_img is True:
            # cv2.imshow is not thread safe. Thats why, I create 2 named window to display camera images
            cv2.namedWindow("left_img", cv2.WINDOW_NORMAL)
            cv2.namedWindow("right_img", cv2.WINDOW_NORMAL)

    def run(self):        
        # ----------------------------------------------------------
        # RESET SCANNER ANGLE
        # ----------------------------------------------------------
        while True:
            self.scanner_pos_pub.publish(self.scanner_pos.data)
            
            print("Scanner angular position(command): ", self.scanner_pos.data)
            print("Scanner angular position(current): ", self.scanner_pos_msg)
            print()
            
            if self.scanner_pos.data + 0.02 >= self.scanner_pos_msg >= self.scanner_pos.data - 0.02:
                break

            self.pcl_pub_rate.sleep()

        # ----------------------------------------------------------
        # RESET ROT TABLE ANGLE
        # ----------------------------------------------------------
        while True:
            self.rot_table_pos_pub.publish(self.rot_table_pos.data)
            
            print("Rot table angular position(command): ", self.rot_table_pos.data)
            print("Rot table angular position(current): ", self.rot_table_pos_msg)
            print()
            
            if self.rot_table_pos.data + 0.02 >= self.rot_table_pos_msg >= self.rot_table_pos.data - 0.02:
                break

            self.pcl_pub_rate.sleep()

        # ----------------------------------------------------------
        # START SCANNING
        # ----------------------------------------------------------
        time.sleep(2)
        print("Scan Started!")
        print()

        for angle in self.rot_table_scan_angles:
            while not rospy.is_shutdown():
                self.rot_table_pos_pub.publish(angle)

                if angle + 0.02 >= self.rot_table_pos_msg >= angle - 0.02:

                    if self.right_cam_img is not None:
                        print("Scan started at angle: ", angle*180/math.pi)
                        self.scan_img(self.left_cam_img, self.right_cam_img)
                        self.publish_point_cloud(self.points)
                        break
                    else:
                        print("File write error")
                
                self.pcl_pub_rate.sleep()

            print("Scanned completed at angle: ", angle*180/math.pi)
            print()
        
        print("Scan completed")
        self.save_point_cloud(self.pcl_transformed, "src/binocular_laser_profile_scanner/scans/scanned_point_cloud")

        # ----------------------------------------------------------
        # PUBLISH POINT CLOUD UNTIL SHUTDOWN
        # ----------------------------------------------------------
        while not rospy.is_shutdown():
            self.pcl_pub.publish(self.pcl_transformed)

    # ----------------------------------------------------------
    # MESSAGE CALLBACKS
    # ----------------------------------------------------------
    def callback_left_cam(self, image_msg):
        try:
            bridge = CvBridge()
            self.left_cam_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as error:
            rospy.logerr(error)

        if self.show_img is True:
            cv2.imshow("left_img", self.right_cam_img)
            cv2.waitKey(1)
    
    def callback_right_cam(self, image_msg):
        try:
            bridge = CvBridge()
            self.right_cam_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as error:
            rospy.logerr(error)

        if self.show_img is True:
            cv2.imshow("right_img", self.right_cam_img)
            cv2.waitKey(1)

    def callback_joint_states(self, joint_state_msg):
        self.scanner_pos_msg = joint_state_msg.position[0]
        self.rot_table_pos_msg = joint_state_msg.position[1]

    # ----------------------------------------------------------
    # SCAN IMAGES FUNCTION
    # ----------------------------------------------------------
    def scan_img(self, left_cam_img, right_cam_img):
        # Laser triangulation formula calculated according to right cam image.
        # Since right and left cam images are symmetrical, 
        # you can just flip left cam image vertically and use same laser triangulation formula.

        # vertical flip
        left_cam_img = cv2.flip(left_cam_img, 0)

        # thinning
        left_cam_img_thinned = thin(left_cam_img)
        right_cam_img_thinned = thin(right_cam_img)

        # finding non zero points
        non_zero_pts_left = cv2.findNonZero(left_cam_img_thinned)
        non_zero_pts_right = cv2.findNonZero(right_cam_img_thinned)

        non_zero_pts = np.concatenate([non_zero_pts_left, non_zero_pts_right])
        non_zero_pts = np.unique(non_zero_pts, axis=0)

        # laser triangulation
        if non_zero_pts is not None:
            self.points = calculate_multiple_pos(non_zero_pts)

    # ----------------------------------------------------------
    # CREATE AND PUBLISH POINT CLOUD FUNCTION
    # ----------------------------------------------------------
    def publish_point_cloud(self, points):
        # creating point cloud
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.pcl_frame_id
        
        point_cloud = pc2.create_cloud_xyz32(header, points)

        # transforming point cloud from laser frame to rot table frame
        self.pcl_transformed = self.transform_point_cloud(point_cloud,
                                                          from_frame=self.pcl_source_frame,
                                                          to_frame=self.pcl_target_frame)

        # publishing transformed point cloud
        self.pcl_pub.publish(self.pcl_transformed)

    # ----------------------------------------------------------
    # TRANSFORM POINT CLOUD
    # ----------------------------------------------------------    
    def transform_point_cloud(self, point_cloud, from_frame, to_frame):
        try:
            # getting transformation matrix from /tf2 topic
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            transform = tf_buffer.lookup_transform(target_frame=to_frame,
                                                   source_frame=from_frame,
                                                   time=rospy.Time(0),
                                                   timeout=rospy.Duration(1.0))
            
            # transforming every point to rot table frame
            for point in pc2.read_points(point_cloud, skip_nans=True):
                p = PointStamped()
                p.header.frame_id = from_frame
                p.point.x = point[0]
                p.point.y = point[1]
                p.point.z = point[2]
                transformed_point = tf_buffer.transform(p, to_frame, rospy.Duration(1.0))
                self.transformed_points.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
            
            # creating point cloud with transformed points
            point_cloud.header.frame_id = to_frame
            new_cloud = pc2.create_cloud_xyz32(point_cloud.header, np.array(self.transformed_points))
            return new_cloud
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cant obtain transformation matrix!")
        
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
            rospy.logerr("File write error")


if __name__ == '__main__':
    scanner = binocular_scanner()
    scanner.run()
