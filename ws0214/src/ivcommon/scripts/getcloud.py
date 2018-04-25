#! /home/jkj/python/anaconda2/bin/python
"""Example of pykitti.odometry usage."""
import itertools
import matplotlib.pyplot as plt
import numpy as np
import rospy
import std_msgs.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2

from mpl_toolkits.mplot3d import Axes3D

import pykitti

#__author__ = "Lee Clement"
#__email__ = "lee.clement@robotics.utias.utoronto.ca"

# Change this to the directory where you store KITTI data
basedir = '/media/jkj/file/dataset/dataset'

# Specify the dataset to load
sequence = '01'

if __name__ == '__main__':
    rospy.init_node('pcl2_pub_example')
    pcl_pub = rospy.Publisher("/my_pcl_topic", PointCloud2)
    # Load the data. Optionally, specify the frame range to load.
    # Passing imformat='cv2' will convert images to uint8 and BGR for
    # easy use with OpenCV.
    # dataset = pykitti.odometry(basedir, sequence)
    dataset = pykitti.odometry(basedir, sequence, frames=range(0, 1000, 1))

    # dataset.calib:      Calibration data are accessible as a named tuple
    # dataset.timestamps: Timestamps are parsed into a list of timedelta objects
    # dataset.poses:      Generator to load ground truth poses T_w_cam0
    # dataset.camN:       Generator to load individual images from camera N
    # dataset.gray:       Generator to load monochrome stereo pairs (cam0, cam1)
    # dataset.rgb:        Generator to load RGB stereo pairs (cam2, cam3)
    # dataset.velo:       Generator to load velodyne scans as [x,y,z,reflectance]
    rospy.loginfo('\nSequence: ' + str(dataset.sequence))
    rospy.loginfo('\nSequence: ' + str(dataset.calib))
    framenum = len(dataset.frames)
    r = rospy.Rate(10)
    poseiter = itertools.islice(dataset.poses, 0, None)
    veloiter = itertools.islice(dataset.velo, 0, None)
    for i in range(framenum):
        # Grab some data
        pose = poseiter.next()
        frame = dataset.frames[i]
        third_velo = veloiter.next()
        timestamp = dataset.timestamps[i]
        # Display some of the data
        np.set_printoptions(precision=4, suppress=True)

        rospy.loginfo('\nFrame : ' + str(frame))


        rospy.loginfo('\ntimestamp: ' + str(timestamp))
        rospy.loginfo('\nground truth pose:\n' + str(pose))

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        #create pcl from points
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1)]
        scaled_polygon_pcl = pcl2.create_cloud(header, fields, third_velo)

        pcl_pub.publish(scaled_polygon_pcl)
        #r.sleep()
        if rospy.is_shutdown():
            break
        """
        f2 = plt.figure()
        ax2 = f2.add_subplot(111, projection='3d')
        # Plot every 100th point so things don't get too bogged down
        velo_range = range(0, third_velo.shape[0], 100)
        ax2.scatter(third_velo[velo_range, 0],
                    third_velo[velo_range, 1],
                    third_velo[velo_range, 2],
                    c=third_velo[velo_range, 3],
                    cmap='gray')
        ax2.set_title('Third Velodyne scan (subsampled)')

        plt.show()
        """
