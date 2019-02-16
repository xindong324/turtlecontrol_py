#!/usr/bin/env python
import math
from picamera.array import PiRGBArray
from picamera import PiCamera
from math import pi
import threading
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import String, UInt8, Int32
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
import time
import cv2

state = {'rotate_to_find_ball': 0, 'follow_and_stop': 1, 'wait_ball_to_stable': 2, 'rotate_self': 3,
         'rotate_around_the_ball': 4, 'rotate_self_to_ball': 5, 'follow_and_push_with_back': 6,'rotate_until_zero':7}
state_value = 0

ball_x = -1
ball_y = -1
ball_r = -1
ball_real_x = 0
ball_real_y = 0
last_x = -1
last_y = -1

last_ball_x = -1
rotate_first = 0
rotate_last = 0
stable_timer = 0
no_ball_timer = 0
rotate_right = 1
rotate=0
control_flag = 0


correct_x = 320
correct_y = 240
correct_r = 180
medium = 320
big_vis = 200
dis_area = 20
small_vis = 150


redLower = np.array([30, 140, 130])
redUpper = np.array([150, 190, 200])
greenLower = np.array([0, 0, 0])
greenUpper = np.array([255, 90, 255])

rospy.init_node('logic_main', anonymous=True)
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(2)
tf_listener = tf.TransformListener()
odom_frame = 'odom'
try:
    tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
    base_frame = 'base_footprint'
except (tf.Exception, tf.ConnectivityException, tf.LookupException):
    try:
        tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_link'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")


def all_control(data):
    global control_flag
    control_flag = data.data
    rospy.loginfo("I have received    " + str(data) + "  !!!!!!!")


pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
sub = rospy.Subscriber('control', Int32, all_control)


def rotate_to_find_ball():
    global state_value
    move_cmd = Twist()
    turn = 1
    if last_ball_x != -1:
        turn = 1 if last_ball_x < 320 else -1
    move_cmd.angular.z = 1 * turn
    move_cmd.linear.x = 0
    if ball_x != -1:
        if abs(rotate) < 0.5:
            state_value = state['rotate_until_zero']
        else:
            state_value = state['follow_and_stop']
    pub.publish(move_cmd)
    rospy.loginfo("rotate_to_find_ball!!!!  z=0.5  ")


def follow_and_stop():
    global state_value, stable_timer, no_ball_timer
    p_angular = 0.004
    p_linear = 0.001
    x_diff = correct_x - ball_x
    r_diff = correct_r - ball_r
    flag_z = 1 if x_diff > 0 else -1
    flag_x = 1 if r_diff > 0 else -1

    if abs(x_diff) < small_vis and abs(r_diff) < dis_area:
        pub_zero()
        state_value = state['wait_ball_to_stable']

    if abs(x_diff) < small_vis:
        x = (min(abs(r_diff), 110)) * flag_x * p_linear
        z = 0
        pub_vel(x, z)

    if abs(x_diff) > big_vis:
        x = 0
        z = (min(abs(x_diff), 200)) * flag_z * p_angular
        pub_vel(x, z)

    if small_vis < abs(x_diff) < big_vis:
        x = (min(abs(r_diff), 110)) * flag_x * p_linear
        z = (min(abs(x_diff), 200)) * flag_z * p_angular
        pub_vel(x, z)

    if ball_x < 0 or ball_y < 0 or ball_r < 0:
        no_ball_timer += 1
        if no_ball_timer > 3:
            state_value = state['rotate_to_find_ball']
            no_ball_timer = 0


def wait_ball_to_stable():
    global state_value, last_x, last_y, ball_real_x, ball_real_y, rotate_first, stable_timer, no_ball_timer
    stable_diff = 20
    unstable_diff = 50
    pub_zero()
    L = 0.3
    if ball_x == -1:
        no_ball_timer += 1
        if no_ball_timer > 3:
            state_value = state['rotate_to_find_ball']
            no_ball_timer = 0
    now_x, now_y = ball_x, ball_y
    if dis(now_x - last_x, now_y - last_y) < stable_diff:
        stable_timer += 1
    if dis(now_x - last_x, now_y - last_y) > unstable_diff:
        stable_timer = 0
    last_x, last_y = now_x, now_y
    if stable_timer > 2:
        rotate_first=rotate
        state_value = state['rotate_self']
    rospy.loginfo("wait ball to stable!!!!!!!!! TIMER is " + str(stable_timer))


def judge_angle(rotate_first, det_angle, acc=0.1):
    global rotate_last
    TarAngle = rotate_first + det_angle
    # vel if don't across 180 degree direction
    errAngle = TarAngle - rotate
    rospy.loginfo("Tarangle " + str(TarAngle * 180.0 / math.pi) + "ROt " + str(rotate * 180.0 / math.pi))
    if abs(TarAngle) > math.pi:
        if TarAngle > 0:
            # from180to-180
            TarAngleMod = TarAngle - 2 * math.pi
            if abs(TarAngleMod - rotate) < acc:
                rotate_last = rotate
                return True
        else:
            # from-180to180
            TarAngleMod = TarAngle + 2 * math.pi
            if abs(rotate - TarAngleMod) < acc:
                rotate_last = rotate
                return True

        z = np.sign(rotate * TarAngleMod * (TarAngleMod - rotate)) * min(abs(TarAngleMod - rotate) / 2.0, 1.5)
        
        ####################modify errAngle##############

        ################################################

    else:
        TarAngleMod = TarAngle
        if abs(rotate - TarAngleMod) < acc:
            rotate_last = rotate
            return True

        z = TarAngleMod - rotate

    if state_value == state['rotate_self'] or state_value == state['rotate_self_to_ball']:
        z = np.sign(z)*max(abs(z),0.2)
        pub_vel(0, z)

    return False


def rotate_self():
    global state_value, rotate_right
    if rotate_first > 0:
        rotate_right = 1  # all on right
        if judge_angle(rotate_first, math.pi / 2):
            state_value = state['rotate_around_the_ball']
    else:
        rotate_right = -1  # all on left
        if judge_angle(rotate_first, -math.pi / 2):
            state_value = state['rotate_around_the_ball']
    rospy.loginfo('rotate_self')


def rotate_around_the_ball():
    global state_value
    pub_vel(0.13, -rotate_right * 0.5)
    rospy.loginfo('rotate_around_the_ball  min  rotate_first=' + str(rotate_first))
    if judge_angle(rotate_last, -rotate_first):
        state_value = state['rotate_self_to_ball']
    rospy.loginfo('rotate_around_the_ball')


def rotate_self_to_ball():
    global state_value, rotate_first
    rospy.loginfo('rotate_self_to_ball')
    if judge_angle(rotate_first, -rotate_right*math.pi / 4):
        if ball_x!=-1:
            if abs(rotate)<0.3:
                state_value = state['rotate_until_zero']
            else:
                state_value = state['follow_and_stop']
        else:
            state_value = state['rotate_to_find_ball']





def rotate_until_zero():
    global state_value
    rospy.loginfo('now i im rotate to zero')
    if abs(rotate)>0.1:
        pub_vel(0,-1.5*np.sign(rotate)*max(abs(rotate),0.2))
    else:
        rospy.loginfo('zero now !!!')
        state_value=state['follow_and_push_with_back']
        



def follow_and_push_with_back():
    global state_value, no_ball_timer
    p_angular = 0.004
    p_linear = 0.001
    the_cmd = Twist()
    x_diff = correct_x - ball_x
    r_diff = 600 - ball_r
    flag_z = 1 if x_diff > 0 else -1x
    flag_x = 1 if r_diff > 0 else -1
    z = (min(abs(x_diff), 300)) * flag_z * p_angular
    x = (min(abs(r_diff), 110)) * flag_x * p_linear
    if abs(z) < 1:
        z = z * 0.5
        x = 0.17
    if abs(z) > 1.3:
        x = -0.05  # back
        z = z / 2.0
    if ball_x < 0 or ball_y < 0 or ball_r < 0:
        no_ball_timer += 1
        if no_ball_timer > 3:
            state_value = state['rotate_to_find_ball']
            no_ball_timer = 0
    the_cmd.angular.z = z
    the_cmd.linear.x = x
    rospy.loginfo("follow_and_push_with_back!!!")
    pub.publish(the_cmd)


def dis(a, b):
    return math.sqrt(a ** 2 + b ** 2)


def pub_zero():
    the_cmd = Twist()
    the_cmd.linear.x = 0
    the_cmd.angular.z = 0
    pub.publish(the_cmd)


def pub_vel(x, z):
    the_cmd = Twist()
    the_cmd.linear.x = x
    the_cmd.angular.z = z
    pub.publish(the_cmd)


def call_back():
    global rotate,state_value
    _,rotate=get_odom()
    if abs(ball_x-medium)<small_vis :
        if 0.1<abs(rotate)<0.3:
            state_value=state['rotate_until_zero']
        if abs(rotate)<0.1:
            state_value=state['follow_and_push_with_back']
    state_to_call[state_value]()


def get_odom():
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), rotation[2])


state_to_call = [rotate_to_find_ball, follow_and_stop, wait_ball_to_stable, rotate_self, rotate_around_the_ball,
                 rotate_self_to_ball, follow_and_push_with_back,rotate_until_zero]

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    ball_x, ball_y, ball_r = -1, -1, -1
    green_flag = 0
    img = frame.array
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    red = cv2.inRange(lab, redLower, redUpper)
    red = cv2.erode(red, None, iterations=8)
    red = cv2.dilate(red, None, iterations=10)
    grass = cv2.inRange(lab, greenLower, greenUpper)
    grass = cv2.erode(grass, None, iterations=2)
    grass = cv2.dilate(grass, None, iterations=10)
    _, contours_grass, _ = cv2.findContours(grass, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours_grass) > 0:
        green_flag = 1
        contours_grass.sort(key=lambda x: len(x))
        grass_cnt = contours_grass[0]
        grass_top = tuple(grass_cnt[grass_cnt[:, :, 1].argmin()][0])
    _, contours_ball, _ = cv2.findContours(red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    res = np.zeros_like(img)
    contours_real_ball = []
    if green_flag == 1:
        for cnt in contours_ball:
            bottom_most = tuple(cnt[cnt[:, :, 1].argmax()][0])
            if bottom_most[1] > grass_top[1]:
                contours_real_ball.append(cnt)
    else:
        contours_real_ball = contours_ball
    if len(contours_real_ball) > 0:
        contours_real_ball.sort(key=lambda x: len(x))
        cnt = contours_real_ball[-1]
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        ball_x, ball_y, ball_r = x, y, radius
        center = (int(x), int(y))
        radius = int(radius)
        print (x, y, radius)
        cv2.circle(res, center, radius, (0, 0, 255), -1)
    rawCapture.truncate(0)
    if control_flag == 2:
        pub_zero()
        state_value=0
        last_ball_x = -1
        rotate_first = 0
        rotate_last = 0
        stable_timer = 0
        no_ball_timer = 0
        rotate_right = 1
        rotate=0
        control_flag = 0
    if control_flag == 1:
        call_back()
    rospy.loginfo(str(control_flag))
   # cv2.imshow("c", res)
  #  cv2.imshow("a",img)

    if cv2.waitKey(100) & 0xFF == ord('q'):
        break

