#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
import signal
import numpy as np
import cv2
# for windows, mac users
# from PIL import ImageGrab
# for linux users
import pyscreenshot as ImageGrab
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def quit(signum, frame):
    print ''
    print 'stop fusion'
    sys.exit()


def screen_cap():
    rospy.init_node('listener', anonymous=True)
    # cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    publisher = rospy.Publisher('screen_image_topic', Image, queue_size=10)
    i = 0
    #change to the window name  you need
    windows = ['Blender', 'Home', 'System Settings']
    lenth = len(windows)
    while True:
        # get the size of the first window, notice for now its just the first!!!!!!!!!!!!!!1
        name = windows[0]
        check_win_name(name)

        rm_above(name)
        add_above(name)

        win_attr = get_win_attr(name)
        x1 = int(win_attr[1])
        y1 = int(win_attr[2])
        x2 = int(win_attr[1]) + int(win_attr[3])
        y2 = int(win_attr[2]) + int(win_attr[4])
        area = (x1, y1, x2, y2)
        # capture computer screen
        img = ImageGrab.grab(bbox=area)
        # img.show()
        # convert image to numpy array
        img_np = np.array(img)
        # convert color space from BGR to RGB
        frame = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)
        immsg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        try:
            publisher.publish(immsg)
        except CvBridgeError as e:
            print(e)

        # show image on OpenCV frame
        # name = '~/save/%d.png' % i
        # cv2.imwrite(name,frame)
        i = i+1
        # cv2.imshow("Screen", frame)
        # write frame to video writer
        time.sleep(0.1)
        if cv2.waitKey(1) == 27:
            break
    cv2.destroyAllWindows()
    # while(cap.isOpened()):
    #     ret, frame = cap.read()
    #     # cv2.imshow('WindowName', frame)
    #     immsg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    #     try:
    #     	publisher.publish(immsg)
    #     except CvBridgeError as e:
    #     	print(e)
    #     if cv2.waitKey(25) & 0xFF == ord('q'):
    #         cap.release()
    #         cv2.destroyAllWindows()
    #         break


if __name__ == '__main__':
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)
    try:
        screen_cap()
    except rospy.ROSInterruptException:
        pass
