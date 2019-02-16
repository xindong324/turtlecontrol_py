#!/usr/bin/env python
# -*- coding: utf-8 -*
import roslib
roslib.load_manifest('video_stream_opencv')
import rospy
from std_msgs.msg import String
from collections import deque
import numpy as np
import time
import cv2
import sys
import signal
from cv_bridge import CvBridge, CvBridgeError
from video_stream_opencv.msg import detect_res
from sensor_msgs.msg import Image
import math
def talker():

    publisher = rospy.Publisher('image_topic_opencv', detect_res, queue_size=10)
    rospy.init_node('web_cam_cp')

    # 设定红色阈值，HSV空间
    redLower = np.array([0, 43, 46])
    redUpper = np.array([15, 255, 255])# 8
    # 设定黑色阈值，HSV空间
    blackLower = np.array([0, 0, 0])
    blackUpper = np.array([180, 255, 46])
    # 初始化追踪点的列表
    mybuffer = 16
    pts = deque(maxlen=mybuffer)
    counter = 0
    # 打开摄像头
    camera = cv2.VideoCapture(0)
    # 等待两秒
    #time.sleep(3)

    while not rospy.is_shutdown():
        # 读取帧
        (ret, frame) = camera.read()
        
        # 判断是否成功打开摄像头
        if not ret:
            print 'No Camera'
            break
        get_label = 'football'
        get_left = 0.0
        get_top = 0.0
        get_right = 0.0
        get_bottom = 0.0
        get_center = []
        # frame = imutils.resize(frame, width=600)
        # 转到HSV空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 根据阈值构建掩膜
        mask = cv2.inRange(hsv, redLower, redUpper)
        # 腐蚀操作
        mask = cv2.erode(mask, None, iterations=2)
        # 膨胀操作，其实先腐蚀再膨胀的效果是开运算，去除噪点
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        # 初始化瓶盖圆形轮廓质心
        center = None
        # 如果存在轮廓
        if len(cnts) > 0:
            # 找到面积最大的轮廓
            c = max(cnts, key=cv2.contourArea)
            # 确定面积最大的轮廓的外接圆
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            # 确定面积最大的轮廓的外接方框
            (x, y, w, h) = cv2.boundingRect(c)
            
            # 计算轮廓的矩
            M = cv2.moments(c)
            # 计算质心
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            max_val = max_w_h(w,h)
            dis_val = dis(max_val)
            # print("have dis :"+str(dis_val))
            
            sqrt = math.sqrt(w*w+h*h)
            print('******************   sqrt: ', sqrt, 'dis_val: ', dis_val)
            # if dis_val <= 40 and sqrt <120: # 50: # 50 is good  55 is no good 50 120
            #     res_msg = detect_res()
            #     res_msg.label=""
            #     res_msg.left = 0.0
            #     res_msg.top = 0.0
            #     res_msg.right = 0.0
            #     res_msg.bottom = 0.0
            #     res_msg.center = [0,0]
            #     publisher.publish(res_msg)
            #     print("don't have sqrt ~~~~~~")
            if dis_val > 50 and sqrt < 30: # 35 ljx  45 55 45

                res_msg = detect_res()
                res_msg.label=""
                res_msg.left = 0.0
                res_msg.top = 0.0
                res_msg.right = 0.0
                res_msg.bottom = 0.0
                res_msg.center = [0,0]
                publisher.publish(res_msg)
                print("don't have sqrt")
            else:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                get_left = x
	        get_top = y
	        get_right = x + w
	        get_bottom = y + h
                get_center.append(center[0])
	        get_center.append(center[1])
	        # 只有当半径大于10时，才执行画图
	        res_msg = detect_res()
	        print("######################   have center:"+str(center))
	        res_msg.label = get_label
	        res_msg.left = get_left
	        res_msg.top = get_top
	        res_msg.right = get_right
	        res_msg.bottom = get_bottom
	        res_msg.center = get_center
	        publisher.publish(res_msg)
        else:
            res_msg = detect_res()
            
            res_msg.label = ""
            res_msg.left = 0.0
            res_msg.top = 0.0
            res_msg.right = 0.0
            res_msg.bottom = 0.0
            res_msg.center = [0,0]
            print("Don`t have center")
            publisher.publish(res_msg)

        #cv2.imshow('Frame', frame)
        # 键盘检测，检测到esc键退出
        k = cv2.waitKey(1) & 0xFF
        #counter += 1
        if k == 27:
            break
    camera.release()
    cv2.destroyAllWindows()
##############################################################################################
def max_w_h(w,h):
    if w>h:
        return w
    else:
        return h     
        
def dis(x):
    y = 1.237e-12 * (x**6) - 2.225e-09 * (x**5) + 1.576e-06 * (x**4) - 0.0005623 *(x**3) +0.1071 * x * x -10.65 * x +498.3 
    return y


if __name__ == '__main__':
    talker()
    


