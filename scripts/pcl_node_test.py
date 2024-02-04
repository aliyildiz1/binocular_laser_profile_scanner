#!/usr/bin/python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import numpy as np

def publish_point_cloud(frame, points):
    rospy.init_node('point_cloud_publisher', anonymous=True)
    pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'base_link'

        cloud_msg = pc2.create_cloud_xyz32(header, points)

        pub.publish(cloud_msg)

        rate.sleep()

if __name__ == '__main__':
    # Creating cube point cloud
    points = []
    
    range_x = np.arange(0.0, 1.0, 0.1)
    range_y = np.arange(0.0, 1.0, 0.1)
    range_z = np.arange(0.0, 1.0, 0.1)

    for x in range_x:
        for y in range_y:
            for z in range_z:
                points.append([x, y, z])
    
    try:
        publish_point_cloud("base_link")
    except rospy.ROSInterruptException:
        pass
