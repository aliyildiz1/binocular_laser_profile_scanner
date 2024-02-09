#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
import math

left_camera_img_topic = "binocular_laser_profile_scanner/left_camera/image_raw"
right_camera_img_topic = "binocular_laser_profile_scanner/right_camera/image_raw"
joint_states_topic = "binocular_laser_profile_scanner/joint_states"

def callback_left_cam(image_msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imshow("left", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as error:
        print(error)

def callback_right_cam(image_msg):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        cv2.imshow("right", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as error:
        print(error)

def callback_joint_states(joint_state_msg):
    rot_table_joint_pos = joint_state_msg.position[0]
    scanner_joint_pos = joint_state_msg.position[1]
    
    print("Rotational table joint pos: ", (scanner_joint_pos*180/math.pi))
    print("Scanner joint pos:          ", (rot_table_joint_pos*180/math.pi))
    

if __name__=="__main__":
    rospy.init_node("scanner_node", anonymous=True)
    
    left_cam_image_subcriber = rospy.Subscriber(left_camera_img_topic, Image, callback_left_cam)
    print("Subscribed images from topic /" + left_camera_img_topic)

    right_cam_image_subcriber = rospy.Subscriber(right_camera_img_topic, Image, callback_right_cam)
    print("Subscribed images from topic /" + right_camera_img_topic)

    joint_state_subcriber = rospy.Subscriber(joint_states_topic, JointState, callback_joint_states)
    print("Subscribed joint states from topic /" + joint_states_topic)

    # cv2.imshow is not thread safe. Thats why, I create 2 named window to display camera images
    cv2.namedWindow("left", cv2.WINDOW_NORMAL)
    cv2.namedWindow("right", cv2.WINDOW_NORMAL)

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")
        cv2.destroyAllWindows()