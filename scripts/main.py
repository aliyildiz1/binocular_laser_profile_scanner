#!/usr/bin/python3

import rospy
import numpy as np
import math
import cv2
import time
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, Float64
from sensor_msgs.msg import PointCloud2, Image, JointState
from cv_bridge import CvBridge, CvBridgeError
from threading import Lock



class binocular_scanner():
    def __init__(self):
        # ----------------------------------------------------------
        # INIT VARIABLES
        # ----------------------------------------------------------
        self.show_img = False

        self.rot_table_pos_msg = 0
        self.scanner_pos_msg = 0

        self.is_rot_table_moving = False
        self.is_scanner_moving = False
        self.is_scan_done = False
        
        self.max_points = 1000000
        self.points = []
        self.pcl_frame_id = "rot_table_link"
        
        # For testing pcl
        range_x = np.arange(0.0, 0.1, 0.02)
        range_y = np.arange(0.0, 0.1, 0.02)
        range_z = np.arange(0.0, 0.1, 0.02)

        for x in range_x:
            for y in range_y:
                for z in range_z:
                    self.points.append([x, y, z])
        
        self.rot_table_pos = Float64()
        self.scanner_pos = Float64()
        
        self.rot_table_pos.data = 359 * math.pi / 180
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
        self.left_cam_image_subcriber = rospy.Subscriber(self.left_cam_topic, Image, self.callback_left_cam)
        print("Subscribed images from topic /" + self.left_cam_topic)

        self.right_cam_image_subcriber = rospy.Subscriber(self.right_cam_topic, Image, self.callback_right_cam)
        print("Subscribed images from topic /" + self.right_cam_topic)

        self.joint_state_subcriber = rospy.Subscriber(self.joint_states_topic, JointState, self.callback_joint_states)
        print("Subscribed joint states from topic /" + self.joint_states_topic)

        # ----------------------------------------------------------
        # INIT PUBLISHERS
        # ----------------------------------------------------------
        self.pcl_pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)
        self.pcl_pub_rate = rospy.Rate(10)  # 1 Hz

        self.rot_table_pos_pub = rospy.Publisher(self.rot_table_pos_cmd_topic, Float64, queue_size=10)
        self.scanner_pos_pub = rospy.Publisher(self.scanner_pos_cmd_topic, Float64, queue_size=10)

        if self.show_img is True:
            # cv2.imshow is not thread safe. Thats why, I create 2 named window to display camera images
            cv2.namedWindow("left", cv2.WINDOW_NORMAL)
            cv2.namedWindow("right", cv2.WINDOW_NORMAL)

    def run(self):
        """
        1) Döner tablaya pozisyon komutu gönderilecek.
        2) Tabla dönmeye başlar başlamaz kameradan alınan görüntüler lazer üçgenleme ile nokta bulutuna çevrilecek
        3) Nokta bulutu 1 Hz ile topic üzerinden yayınlanacak
        4) Tabla 360 derece döndüğünde nokta bulutu kaydedilecek ve program sonlandırılacak 
        """
        
        # 1 - Publish commands
        while True:
            self.scanner_pos_pub.publish(self.scanner_pos.data)
            self.is_scanner_moving = True
            
            print("Scanner angular position(command): ", self.scanner_pos.data)
            print("Scanner angular position(current): ", self.scanner_pos_msg)
            print()
            
            if self.scanner_pos_msg >= self.scanner_pos.data - 0.02:
                self.is_scanner_moving = False
                break

            self.pcl_pub_rate.sleep()

        print("Scan Started!")
        while not rospy.is_shutdown():
            self.rot_table_pos_pub.publish(self.rot_table_pos.data)
            self.is_rot_table_moving = True
            
            self.publish_point_cloud()

            print("Rot table position(command): ", self.rot_table_pos.data)
            print("Rot table position(current): ", self.rot_table_pos_msg)
            print()
            
            if self.rot_table_pos_msg >= self.rot_table_pos.data - 0.001:
                self.is_rot_table_moving = False
                self.is_scan_done = True
                print("Scan completed!")
                break

            self.pcl_pub_rate.sleep()

    def callback_left_cam(self, image_msg):
        pass
    
    def callback_right_cam(self, image_msg):
        try:
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            if self.show_img is True:
                cv2.imshow("right", cv_image)
                cv2.waitKey(1)
        except CvBridgeError as error:
            print(error)
        
        # laser triangulation


    def callback_joint_states(self, joint_state_msg):
        with self.msg_lock:
            self.scanner_pos_msg = joint_state_msg.position[0]
            self.rot_table_pos_msg = joint_state_msg.position[1]

    def publish_point_cloud(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.pcl_frame_id
        
        with self.msg_lock:
            cloud_msg = pc2.create_cloud_xyz32(header, self.points)

        self.pcl_pub.publish(cloud_msg)


if __name__ == '__main__':
    scanner = binocular_scanner()
    scanner.run()
