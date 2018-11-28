#!/usr/bin/env python
# -*- coding=utf-8 -*-


"""
file: txtSender.py
socket client
"""
import rospy
import socket
import os
import sys
import struct
import time
import signal
import pyproj
import numpy as np
import os

def quit(signum, frame):
    print ''
    print 'stop fusion'
    sys.exit()

def socket_client():
    # try:
    #     s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #     #sk.connect_ex(address)
    #     s.connect_ex(('192.168.43.232', 6666))
    # except socket.error as msg:
    #     print msg
    # rospy.init_node("txtsender")
    # port = rospy.get_param('~port')
    # ip = rospy.get_param('ip')
    path = os.path.expanduser('~')
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while s.connect_ex(("192.168.10.50  ", 6666)) != 0:
        print "wating for vehicle to open server ..."
        time.sleep(1)


    print s.recv(1024)

    while 1:
        filepath = path + '/taskfile/KYXZ2018A.txt'
        if os.path.isfile(filepath):
            # 定义定义文件信息。128s表示文件名为128bytes长，l表示一个int或log文件类型，在此为文件大小
            fileinfo_size = struct.calcsize('128sl')
            # 定义文件头信息，包含文件名和文件大小
            fhead = struct.pack('128sl', os.path.basename(filepath),
                                os.stat(filepath).st_size)
            s.send(fhead)
            print 'client filepath: {0}'.format(filepath)

            fp = open(filepath, 'rb')
            while 1:
                data = fp.read(1024)
                if not data:
                    print '{0} file send over...'.format(filepath)
                    break
                s.send(data)
        s.close()
        break
    sys.exit('sending over')
def readTXT(fileName):
    print('#########')
    points = []
    with open(fileName, 'r') as f:
        for line in f.readlines():
            line = line.strip().split(' ')
            # line[0] is task point's number, multiple 10 for adding point
            # added point's name should plus 1
            point = [int(line[0]), float(line[1]), float(line[2]), float(line[3]), int(line[4])]
            points.append(point)
            print(point)

    return points


def GCS2PCS(points, standard):
    print('#########')
    p1 = pyproj.Proj(init="epsg:4326")
    p2 = pyproj.Proj(init="epsg:3857")

    print("standard point {0}".format(standard))
    standard_proj = standard
    standard_proj = np.array(standard_proj)
    standard_proj[0], standard_proj[1] = pyproj.transform(
        p1, p2, standard_proj[0], standard_proj[1])

    points_new = []

    count = 1
    for each in points:
        if each[-1] == 3:
            print('find type 3')
            point = [each[1], each[2]]
            point = np.array(point)
            point[0], point[1] = pyproj.transform(p1, p2, point[0], point[1])
            dist = np.linalg.norm(point - standard_proj)
            print(dist)
            if dist <= 90:
                point_temp1 = [count, 116.100695, 40.155812, 0.0, 2]
                points_new.append(point_temp1)
                count += 1
                point_temp1 = [count, 116.09951102, 40.15483262, 0.0, 2]
                points_new.append(point_temp1)
                count += 1
                point_temp1 = [count, 116.09912492, 40.15564674, 0.0, 2]
                points_new.append(point_temp1)
                count += 1
                each[1] = standard[0]
                each[2] = standard[1]
                print('<=90')

        each[0] = count
        points_new.append(each)
        count += 1

    return points_new


def writeTXT():
	home = os.path.expanduser('~')
	fileName = home + '/taskfile/KYXZ2018A.txt'
	standard = [116.099804, 40.155690]
	points = readTXT(fileName)
	points_new = GCS2PCS(points, standard)

	print('#########')
	with open(fileName, 'w') as f:
	    for each in points_new:
	        line = '%d %.8f %.8f %.8f %d\n' % (int(each[0]), float(
	            each[1]), float(each[2]), float(each[3]), int(each[4]))
	        f.write(line)

	print('write done')

if __name__ == '__main__':


    # fileName = home + '/taskfile/KYXZ2018A.txt'
    writeTXT()
    socket_client()