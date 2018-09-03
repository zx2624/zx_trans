
#!/usr/bin/env python
# -*- coding:utf-8 -*- 
import rospy
from std_msgs.msg import String
import sys
import os.path
from cv_bridge import CvBridge
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

def callback(data):
    print("-------------have successful get data:%s"%data)

def get_data():
    rospy.init_node('get_data', anonymous=True)
    rospy.Subscriber('road_type', String, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
	print "sldkffja;ldfkjsldf"
    get_data()
