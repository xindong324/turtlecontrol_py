#!/usr/bin/env python
import math
from math import pi
import threading
import os
import signal
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge 
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
#from darknet_ros_msgs.msg import BoundingBox
#from darknet_ros_msgs.msg import BoundingBoxes
import time
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String



def get_odom():
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), rotation[2])

def rotate_angle(rotate_first,det_angle):
    
    the_cmd=Twist()
    the_cmd.linear.x=0
    the_cmd.angular.z=-1
    TarAngle = rotate_first + det_angle

    (_, rotate) = get_odom()

    #vel if don't across 180 degree direction
   
    rospy.loginfo("Tarangle " + str(TarAngle*180.0/math.pi)+"ROt " + str(rotate*180.0/math.pi))
    if abs(TarAngle)>math.pi:
        if TarAngle>0:
            #from180to-180
            TarAngle = TarAngle-2*math.pi
            if abs(TarAngle-rotate)<0.1:
                
                return True
        else:
            #from-180to180
            TarAngle = TarAngle+2*math.pi
            if abs(rotate - TarAngle)<0.1:
                return True
    else:
            if abs(rotate - TarAngle)<0.1:
                return True

    if det_angle>0 :
      velAng = min(abs(TarAngle - rotate),1.5)
    else:
      velAng = -1*min(abs(TarAngle - rotate),1.5)
   # TarAngle-rotate
   # cmdVel = Twist()
    the_cmd.angular.z = velAng
    the_cmd.linear.x = 0
    rospy.loginfo("Velangle " + str(velAng))
    pub.publish(the_cmd)
    return False


def judge_angle(rotate_first,det_angle,acc=0.1):
    (_, rotate) = get_odom()
    TarAngle = rotate_first + det_angle
    #vel if don't across 180 degree direction
    errAngle = TarAngle - rotate

    the_cmd=Twist()
    rospy.loginfo("Tarangle " + str(TarAngle*180.0/math.pi)+"ROt " + str(rotate*180.0/math.pi))
    if abs(TarAngle)>math.pi:


        if TarAngle>0:
            #from180to-180
            TarAngleMod = TarAngle-2*math.pi
            if abs(TarAngleMod-rotate)<acc:
                rotate_last=rotate
                return True
        else:
            #from-180to180
            TarAngleMod = TarAngle+2*math.pi
            if abs(rotate - TarAngleMod)<acc:
                rotate_last=rotate
                return True

        ####################modify errAngle##############
        if fabs(TarAngle-rotate)<0 :
            #rotate are in same side with target
            rospy.loginfo("TestMod")
            errAngle = TarAngleMod - rotate
        
        ################################################
        the_cmd.angular.z = np.sign(rotate*(TarAngleMod - rotate))*min(abs(TarAngleMod - rotate),1.5)
    else:
        errAngle = TarAngle - rotate
        TarAngleMod = TarAngle
        the_cmd.angular.z = TarAngleMod - rotate
        if abs(rotate - TarAngleMod)<acc:
            rotate_last=rotate
            return True

        
    rospy.loginfo("errAng"+str(errAngle))
    
    the_cmd.linear.x=0
    #the_cmd.angular.z = np.sign(rotate*(TarAngleMod - rotate))*min(abs(TarAngleMod - rotate),1.5)
    pub.publish(the_cmd)
    #pub_vel(0,np.sign(rotate*(TarAngleMod - rotate))*(abs(errAngle),1.5))
         
    return False


def move2Point(TargetX,TargetY):
   # (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        (goal_x, goal_y, goal_z) = self.getkey()
        if goal_z > 180 or goal_z < -180:
            print("you input worng z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        if distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
           # r.sleep()
def rotate2Angle(TarAngle)
    (position, rotation) = self.get_odom()

    if abs(rotation - goal_z) > 0.05:
        #(position, rotation) = self.get_odom()
        if goal_z >= 0:
            if rotation <= goal_z and rotation >= goal_z - pi:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = 0.5
            else:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = -0.5
        else:
            if rotation <= goal_z + pi and rotation > goal_z:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = -0.5
            else:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = 0.5
        self.cmd_vel.publish(move_cmd)


if __name__ == '__main__':
    rospy.init_node('rotate', anonymous=True)
    state = { 'push_the_ball': 0, 'find_ball': 1}
    #state_to_call = [ push_the_ball, find_ball]
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
   # img_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback, queue_size = 1)
    state_value = 0
    gap = 10
    turn = 1
    to_spot_flag=1
    medium = 320
    biase1 = 50
    biase2 = 160
    goal_x, goal_y, goal_z= 1.20, 0.55, 0
    ball_vis_x=-1
    ball_vis_y=-1
    ball_vis_z=-1
    r = rospy.Rate(10)
    tf_listener = tf.TransformListener()
    global odom_frame
    odom_frame = 'odom'
    vel_cmd = Twist()
    try:
        tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_footprint'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        try:
            tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
            base_frame = 'base_link'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
    (_,rotate_first) = get_odom()

    while judge_angle(0,-174*math.pi/180.0)!=True :
       rospy.loginfo("gg")
       r.sleep()

    vel_cmd = Twist()
    pub.publish(vel_cmd)
        
   # rospy.spin()