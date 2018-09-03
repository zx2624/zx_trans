#!/usr/bin/env python
# -*- coding=utf-8 -*-


"""
file: txtReceiver.py
socket service
"""

import rospy
import socket
import threading
import time
import sys
import os
import struct



def socket_service():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('192.168.1.102', 6666))
        s.listen(10)
    except socket.error as msg:
        print(msg)
        sys.exit(1)
    print('Waiting connection...')

    while 1:
        conn, addr = s.accept()
        deal_data(conn, addr)
        # t = threading.Thread(target=deal_data, args=(conn, addr))
        # t.start()

def deal_data(conn, addr):
    print('Accept new connection from {0}'.format(addr))
    #conn.settimeout(500)
    conn.send('Hi, Welcome to the server!')

    if 1:
        fileinfo_size = struct.calcsize('128sl')
        buf = conn.recv(fileinfo_size)
        if buf:
            filename, filesize = struct.unpack('128sl', buf)
            fn = filename.strip('\00')
            # new_filename = os.path.join('./', + fn)

            recvd_size = 0  # 定义已接收文件的大小
            newname = "/home/zx/test/ws_zx_trans/src/task_sender/scripts/"+fn
            print('file new name is {0}, filesize is {1}'.format(newname,
                                                                 filesize))
            fp = open(newname, 'wb')
            print('start receiving...')

            while not recvd_size == filesize:
                if filesize - recvd_size > 1024:
                    data = conn.recv(1024)
                    recvd_size += len(data)
                else:
                    data = conn.recv(filesize - recvd_size)
                    recvd_size = filesize
                fp.write(data)
            fp.close()
            # print('end receive...')
        conn.close()
        sys.exit('end receive...')
        # break


if __name__ == '__main__':
	# global port 
	# port = 6666
	# port = rospy.get_param("port")
	# print "============port=========%d" % port
    socket_service()