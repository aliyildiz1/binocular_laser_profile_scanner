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

        self.rot_table_angles = np.arange(0.0, 6.4, 0.1)
        self.rot_table_pos_msg = 0
        self.scanner_pos_msg = 0
        
        self.max_points = 1000000
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
        #self.left_cam_image_subcriber = rospy.Subscriber(self.left_cam_topic, Image, self.callback_left_cam, queue_size = 1, buff_size=2**19)
        #print("Subscribed images from topic /" + self.left_cam_topic)

        self.right_cam_image_subcriber = rospy.Subscriber(self.right_cam_topic, Image, self.callback_right_cam, queue_size = 1, buff_size=2**8)
        print("Subscribed images from topic /" + self.right_cam_topic)

        self.joint_state_subcriber = rospy.Subscriber(self.joint_states_topic, JointState, self.callback_joint_states)
        print("Subscribed joint states from topic /" + self.joint_states_topic)

        # ----------------------------------------------------------
        # INIT PUBLISHERS
        # ----------------------------------------------------------
        self.pcl_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=1)
        self.pcl_pub_rate = rospy.Rate(1)  # 1 Hz

        self.rot_table_pos_pub = rospy.Publisher(self.rot_table_pos_cmd_topic, Float64, queue_size=1)
        self.scanner_pos_pub = rospy.Publisher(self.scanner_pos_cmd_topic, Float64, queue_size=1)

        if self.show_img is True:
            # cv2.imshow is not thread safe. Thats why, I create 2 named window to display camera images
            # cv2.namedWindow("left_img", cv2.WINDOW_NORMAL)
            cv2.namedWindow("right_img", cv2.WINDOW_NORMAL)

    def run(self):        
        # Reset scanner angle
        while True:
            self.scanner_pos_pub.publish(self.scanner_pos.data)
            self.is_scanner_moving = True
            
            print("Scanner angular position(command): ", self.scanner_pos.data)
            print("Scanner angular position(current): ", self.scanner_pos_msg)
            print()
            
            if self.scanner_pos.data + 0.02 >= self.scanner_pos_msg >= self.scanner_pos.data - 0.02:
                self.is_scanner_moving = False
                break

            self.pcl_pub_rate.sleep()

        # Reset rot table angle
        while True:
            self.rot_table_pos_pub.publish(self.rot_table_pos.data)
            self.is_scanner_moving = True
            
            print("Rot table angular position(command): ", self.rot_table_pos.data)
            print("Rot table angular position(current): ", self.rot_table_pos_msg)
            print()
            
            if self.rot_table_pos.data + 0.02 >= self.rot_table_pos_msg >= self.rot_table_pos.data - 0.02:
                self.is_scanner_moving = False
                break

            self.pcl_pub_rate.sleep()

        print("Scan Started!")
        print(self.rot_table_angles)
        for angle in self.rot_table_angles:
            while not rospy.is_shutdown():
                self.rot_table_pos_pub.publish(angle)

                if angle + 0.02 >= self.rot_table_pos_msg >= angle - 0.02:
                    # print("Current rot table angle: ", angle*180/math.pi)

                    time.sleep(0.5)
                    if self.right_cam_img is not None:
                        self.scan_img(self.right_cam_img)
                        self.publish_point_cloud()
                        break
                    else:
                        print("right cam img is None")
                
                self.pcl_pub_rate.sleep()

            print("Scanned rot table angle: ", angle*180/math.pi)

    def callback_left_cam(self, image_msg):
        pass
    
    def callback_right_cam(self, image_msg):
        try:
            bridge = CvBridge()
            self.right_cam_img = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as error:
            print(error)

        if self.show_img is True:
            cv2.imshow("right_img", self.right_cam_img)
            cv2.waitKey(1)


    def callback_joint_states(self, joint_state_msg):
        self.scanner_pos_msg = joint_state_msg.position[0]
        self.rot_table_pos_msg = joint_state_msg.position[1]

    def publish_point_cloud(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.pcl_frame_id
        
        point_cloud = pc2.create_cloud_xyz32(header, self.points)

        self.pcl_transformed = self.transform_point_cloud(point_cloud,
                                                          from_frame=self.pcl_source_frame,
                                                          to_frame=self.pcl_target_frame)

        self.pcl_pub.publish(self.pcl_transformed)

    def scan_img(self, img):
        # laser triangulation
        thinned_img = thin(img)
        non_zero_pts = cv2.findNonZero(thinned_img)

        if non_zero_pts is not None:
            with self.msg_lock:
                self.points = calculate_multiple_pos(non_zero_pts)
    
    def transform_point_cloud(self, point_cloud, from_frame, to_frame):
        try:
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            transform = tf_buffer.lookup_transform(target_frame=to_frame,
                                                   source_frame=from_frame,
                                                   time=rospy.Time(0),
                                                   timeout=rospy.Duration(1.0))
            
            for point in pc2.read_points(point_cloud, skip_nans=True):
                p = PointStamped()
                p.header.frame_id = from_frame
                p.point.x = point[0]
                p.point.y = point[1]
                p.point.z = point[2]
                transformed_point = tf_buffer.transform(p, to_frame, rospy.Duration(1.0))
                self.transformed_points.append([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
            
            point_cloud.header.frame_id = to_frame
            new_cloud = pc2.create_cloud_xyz32(point_cloud.header, np.array(self.transformed_points))
            return new_cloud       
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cant obtain transformation matrix!")
        
        return None

if __name__ == '__main__':
    scanner = binocular_scanner()
    scanner.run()
