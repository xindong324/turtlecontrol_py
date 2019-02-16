#!/usr/bin/env python
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
from math import pi
import threading
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import cv2


state={'rotate_to_find_ball':0,'follow_and_stop':1,'wait_ball_to_stable':2,'rotate_self':3,'rotate_around_the_ball':4,'rotate_self_to_x_axis':5,'follow_and_push_with_back':6}
state_value=0



ball_x=-1
ball_y=-1
ball_r=-1
ball_real_x=0
ball_real_y=0
last_x=-1
last_y=-1
correct_x=320
correct_y=240
correct_r=200
last_ball_x=-1
rotate_first=0
stable_timer=0
no_ball_timer=0



redLower = np.array([30, 150, 130])
redUpper = np.array([100, 200, 170])
greenLower = np.array([0, 0, 170])
greenUpper = np.array([255, 90, 200])





rospy.init_node('logic_main',anonymous=True)
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(2)
pub=rospy.Publisher('cmd_vel',Twist,queue_size=1)
tf_listener=tf.TransformListener()
odom_frame='odom'
try:
    tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    base_frame = 'base_footprint'
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")


def rotate_to_find_ball():
    global state_value
    move_cmd = Twist()
    turn =1
    if last_ball_x!=-1:
        turn=1 if  last_ball_x<320 else -1
    move_cmd.angular.z =  0.5*turn
    move_cmd.linear.x=0
    (position, rotation) = get_odom()
    if ball_x!=-1 :
        if abs(rotation)<0.5:
            state_value=state['follow_and_push_with_back']
        else:
            state_value=state['follow_and_stop']
    pub.publish(move_cmd)
    rospy.loginfo("rotate_to_find_ball!!!!  z=0.5  ")




def follow_and_stop():
    global state_value,stable_timer,no_ball_timer
    p_angular=0.004
    p_linear=0.001
    the_cmd=Twist()
    x_diff=correct_x-ball_x
    r_diff=correct_r-ball_r
    flag_z=1 if x_diff>0 else -1
    flag_x=1 if r_diff>0 else -1
    z=(min(abs(x_diff),200))*flag_z*p_angular
    x=(min(abs(r_diff),110))*flag_x*p_linear
    if abs(x)<0.04 and abs(z)<0.2:
        print "wait_ball_to_stable_now!!!!"
        stable_timer=0
        state_value=state['wait_ball_to_stable']
        z=0
        x=0
    elif abs(z)<0.2 and x>0.1:
        z=0
        x=0.15
    else:        
        if abs(x)<0.04:
            x=0
        if abs(z)>0.2:
            x=0
        if ball_x<0 or ball_y<0 or ball_r<0:
            no_ball_timer+=1
            if no_ball_timer>3:
                state_value=state['rotate_to_find_ball']
                no_ball_timer=0            
    the_cmd.angular.z=z
    the_cmd.linear.x=x
    rospy.loginfo("follow_and_stop!!!publish the z ="+str(z)+" the x = "+str(x)+" x_diff = "+str(x_diff)+" r_diff " +str(r_diff)+" ball_x="+str(ball_x)+" ball_y"+str(ball_y)+" ball_r "+str(ball_r))
    pub.publish(the_cmd)



def dis(a,b):
    return math.sqrt(a**2+b**2)

def wait_ball_to_stable():
    global state_value,last_x,last_y,ball_real_x,ball_real_y,rotate_first,stable_timer
    stable_diff=20
    unstable_diff=50
    L=0.2
    if ball_x==-1:
        no_ball_timer+=1
        if no_ball_timer>3:
            state_value=state['rotate_to_find_ball']
            no_ball_timer=0    
    (position, rotation) = get_odom()
    now_x,now_y=ball_x,ball_y
    if dis(now_x-last_x,now_y-last_y)<stable_diff:
        stable_timer+=1
    if dis(now_x-last_x,now_y-last_y)>unstable_diff:
        stable_timer=0
    last_x,last_y=now_x,now_y
    if stable_timer>2:
        (position, rotate_first) = get_odom()
        ball_real_x=position.x+L*math.cos(rotate_first)
        ball_real_y=position.y+L*math.sin(rotate_first)
        state_value=state['rotate_self']
    rospy.loginfo("wait ball to stable!!!!!!!!! TIMER is "+str(stable_timer))



def rotate_self():
    global state_value
    the_cmd=Twist()
    the_cmd.linear.x=0
    the_cmd.angular.z=-1
    (position, rotate) = get_odom()
    if math.pi/2<rotate_first<math.pi:
        if abs(abs(-1*math.pi/2+(rotate-rotate_first))-math.pi/2)<0.3:
            state_value=state['rotate_around_the_ball']
    else:
        if abs(abs(rotate-rotate_first)-math.pi/2)<0.3:
            state_value=state['rotate_around_the_ball']
    rospy.loginfo("rotate_self!!!rotate= "+str(rotate)+"rotate_first="+str(rotate_first)+"x=0,z=1")

    pub.publish(the_cmd)



def rotate_around_the_ball():
    global state_value
    the_cmd=Twist()
    the_cmd.linear.x=-0.1
    the_cmd.angular.z=-0.4
    (position, rotation) = get_odom()
    if position.x<ball_real_x and abs(position.y-ball_real_y)<0.1:
        state_value=state['rotate_self_to_x_axis']
    rospy.loginfo("rotate_around_the_ball!!!position.x= "+str(position.x)+"ball_real_x="+str(ball_real_x)+"x= -0.1,z=1")
    pub.publish(the_cmd)


def rotate_self_to_x_axis():
    global state_value
    the_cmd=Twist()
    the_cmd.linear.x=0
    the_cmd.angular.z=-1
    (position, rotation) = get_odom()
    if abs(rotation)<0.3:
        state_value=state['follow_and_push_with_back']
    rospy.loginfo("rotate_self_to_x_axis!!!  rotation= "+str(rotation)+"x= 0  , z = 1")
    pub.publish(the_cmd)


def follow_and_push_with_back():
    global state_value,no_ball_timer
    p_angular=0.004
    p_linear=0.001
    the_cmd=Twist()
    x_diff=correct_x-ball_x
    r_diff=600-ball_r
    flag_z=1 if x_diff>0 else -1
    flag_x=1 if r_diff>0 else -1
    z=(min(abs(x_diff),300))*flag_z*p_angular
    x=(min(abs(r_diff),110))*flag_x*p_linear
    if abs(z)<0.5:
        z=z*0.5
        x=0.15
    if abs(z)>0.7:
        x=-0.07 #back
        z=z/2.0
    if ball_x<0 or ball_y<0 or ball_r<0:
        no_ball_timer+=1
        if no_ball_timer>3:
            state_value=state['rotate_to_find_ball']
            no_ball_timer=0
    the_cmd.angular.z=z
    the_cmd.linear.x=x
    rospy.loginfo("follow_and_push_with_back!!!publish the z ="+str(z)+" the x = "+str(x)+" x_diff = "+str(x_diff)+" r_diff " +str(r_diff)+" ball_x="+str(ball_x)+" ball_y"+str(ball_y)+" ball_r "+str(ball_r))
    pub.publish(the_cmd)


    

    



def get_odom():
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), rotation[2])
 


state_to_call=[rotate_to_find_ball,follow_and_stop,wait_ball_to_stable,rotate_self,rotate_around_the_ball,rotate_self_to_x_axis,follow_and_push_with_back]

for frame in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):
    ball_x,ball_y,ball_r=-1,-1,-1
    img = frame.array
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    red = cv2.inRange(lab, redLower, redUpper)
    red = cv2.erode(red, None, iterations=8)
    red = cv2.dilate(red, None, iterations=10)
    grass = cv2.inRange(lab, greenLower, greenUpper)
    grass = cv2.erode(grass, None, iterations=2)
    grass = cv2.dilate(grass, None, iterations=10)
    contours_grass,_= cv2.findContours(grass, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours_ball, _ = cv2.findContours(red, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    res=np.zeros_like(img)
    if len(contours_ball) > 0:
        contours_ball.sort(key=lambda x: len(x))
        cnt = contours_ball[-1]
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        if y>100:
            if len(contours_grass) > 0:
                contours_grass.sort(key=lambda x: len(x))
                grass_cnt = contours_grass[0]
                cv2.drawContours(img, grass_cnt, -1, (0, 255, 0), 2)
                (x_g, y_g), (w_g, h_g), _ = cv2.minAreaRect(grass_cnt)
                if y+radius>y_g-h_g/2:
                    print (x, y, radius)
                    ball_x, ball_y = x, y
                    last_ball_x=x
                    center = (int(x), int(y))
                    radius = int(radius)
                    cv2.circle(res, center, radius, (0, 0, 255), -1)
            else:
                ball_x, ball_y = x, y
                last_ball_x=x
                center = (int(x), int(y))
                radius = int(radius)
                cv2.circle(res, center, radius, (0, 0, 255), -1)
    cv2.imshow("c", res)
    cv2.imshow("a",img)

    if cv2.waitKey(100) & 0xFF == ord('q'):
        break


def updateVel(topic, velLine,velAng)
    move_cmd = Twist()
    move_cmd.linear.x = velLine
    move_cmd.angular.z = velAng

    pub.publish()