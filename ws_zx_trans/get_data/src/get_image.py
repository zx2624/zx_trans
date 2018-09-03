#!/usr/bin/env python
#-*-coding:utf-8-*-

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import json
from PIL import Image as IImage
import logging
import commentjson
import collections
import cv2  
import numpy as np
import sys
import scipy as scp
import scipy.misc
import tensorflow as tf
from tensorflow.python.platform import gfile
import os.path
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'


def callback(data):
	bridge=CvBridge()
	img_raw=bridge.imgmsg_to_cv2(data,"bgr8")#图像是uint８格式

	
	#######--------------------------------------------------------------######
	#b, g, r = img_raw.split()
	#image = IImage.merge("RGB", (r, g, b))

	#######--------------------------------------------------------------######
	#img_raw1=img_raw.astype(np.float32)
	#b, g, r = img_raw1.split()
	#image = IImage.merge("RGB", (r, g, b))

	####----------------------------------------------------------------######
	#data = np.asarray(img_raw)
	#image = IImage.fromarray(np.roll(data, 1, axis=-1))

	####----------------------------------------------------------------######
	#data = np.asarray(img_raw)
	#data1=data.astype(np.float32)
	#b, g, r = data1.split()
	#image = IImage.merge("RGB", (r, g, b))

	####----------------------------------------------------------------######
	image = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)


	global image_dir,video_dir,videoWriter
	#轉換圖像大小
	#img=tf.image.resize_images(img,(480,960),0)
	#image=sess.run(img)

	#cv2.imshow("green_image",green_image)
	global count
	#new_name = name.split('_')[0] + "_road_" + name.split('_')[1]
	new_name = "%s"%count + ".jpg"
	save_file = os.path.join(image_dir, new_name)
	scp.misc.imsave(save_file, image)
	#save_file = os.path.join(raw_image_softmax, new_name)
	#scp.misc.imsave(save_file, output_im)
	#save_file = os.path.join(raw_image_rb, new_name)
	#scp.misc.imsave(save_file, ov_image)

	videoWriter.write(image)
	output_string="The images and video successfully write"
	count+=1
	try:
		pub = rospy.Publisher('get_image', String, queue_size=10)
		#rospy.init_node('road_identify', anonymous=True) 
		rate = rospy.Rate(10) # 10hz
		#while not rospy.is_shutdown():
		rospy.loginfo(output_string)
		pub.publish(output_string)
		rate.sleep()
	except rospy.ROSInterruptException:
		pass
	# #写入图片
	# img=tf.image.convert_image_dtype(img,dtype=tf.uint8)
	# img=tf.image.encode_jpeg(img)
	# with tf.gfile.GFile('./trans/%s.jpg'%(i+1),'wb') as f:
		# f.write(img.eval())
def main(_):
	global image_dir,video_dir,videoWriter
	#current_dir=os.getcwd()#当前路径,ros在/home/dzl/catkin_ws下調用concreteroad.py，當前路勁爲/home/dzl/catkin_ws
	current_dir='/home/dzl/catkin_ws/src/get_data/src'

	image_dir = os.path.join(current_dir,'image')
	if not os.path.exists(image_dir):
		os.makedirs(image_dir)
	video_dir = os.path.join(current_dir,'video')
	if not os.path.exists(video_dir):
		os.makedirs(video_dir)
	#hypes['dirs']['image_dir'] =image_dir
	#hypes['dirs']['video_dir'] =video_dir

	fps=15
	size=(960,480)
	video=os.path.join(video_dir,'raw.avi')
	videoWriter = cv2.VideoWriter(video, cv2.VideoWriter_fourcc('I','4','2','0'), fps, size)

	global count
	count=0

	rospy.init_node('get_image', anonymous=True)
	rospy.Subscriber('image', Image, callback)
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	videoWriter.release()
	cv2.destroyAllWindows()
if __name__ == '__main__':
	tf.app.run()
