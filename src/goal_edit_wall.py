#!/usr/bin/env python
# -*- coding: utf-8 -*
import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np
from numpy import pi
from std_msgs.msg import String
from video_stream_opencv.msg import detect_res
import time
import math


class RES(object):
    get_label = 0
    get_left = 0.0
    get_top = 0.0
    get_right = 0.0
    get_bottom = 0.0
    get_center = []
    def __init__(self):
        print("initing~~~~~")

    def callback(self, data):
        self.get_label = data.label
        self.get_left = data.left
        self.get_top = data.top
        self.get_right = data.right
        self.get_bottom = data.bottom
        self.get_center = data.center

    def listener(self):
        #rospy.init_node('image_rec_listener_two', anonymous=True)
        rospy.Subscriber("image_topic_opencv", detect_res,self.callback)

    def get_img_res(self):
        #!!!!!!!!!!!!这里可以写图像再次处理函数!!!!!!!!!!!!!!#
        return self.get_label, self.get_left, self.get_top, self.get_right, self.get_bottom, self.get_center




class TurtlebotNavigation():
    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        position = Point()
        self.move_cmd = Twist()
        self.r = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.GOAL = False # a global variable: True: goal() False: findBall
        self.counter = 0 # a tempera counter in goal()
        self.DEBUG = False #True
        self.FIRST = True# False # True # to goal at 1st time
        self.BACK = False# True

        #!!!!!!!!!!!!!全局定位参数!!!!!!!!!!!!!!!!!#
        #静态参数
        self.ballDoor_our = [31.5, 85]      #我们球门的中心点
        self.ballDoor_enemy = [260, 85]     #对方球门的中心点
        self.ballDoor_width = 70            #球门宽度
        self.width = 170                    #场地宽度
        self.lone = 291.5                   #场地长度
        self.safety_x =[31.5, 260]          #中间无障碍场地x宽度
        self.safety_y = [5, 165]            #中间无障碍场地y宽度
        #动态参数
        self.init_pose_x = 10
        self.init_pose_y = 10


        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])


    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)




    #!!!!!!!!!!!!!!!!!!功能函数!!!!!!!!!!!!!!!!!!!!!!!#
    def penalty(self, go_x, go_y, go_z):
        # the pose of the car
        (position, rotation) = self.get_odom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        goal_x = go_x
        goal_y = go_y
        goal_z = go_z
        if goal_z > 180 or goal_z < -180:
            print("you input worng z range.")
            self.shutdown()
        goal_z = np.deg2rad(goal_z)
        goal_distance = sqrt(pow(goal_x, 2) + pow(goal_y, 2))
        distance = goal_distance
        position_init_x = position.x
        position_init_y = position.y

        while distance > 0.05:
            (position, rotation) = self.get_odom()
            x_start = position.x
            y_start = position.y
            path_angle = atan2(goal_y - (y_start - position_init_y), goal_x- (x_start - position_init_x))

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle

            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            self.move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - (x_start - position_init_x)), 2) + pow((goal_y - (y_start - position_init_y)), 2))
            print(distance)
            self.move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if self.move_cmd.angular.z > 0:
                self.move_cmd.angular.z = min(self.move_cmd.angular.z, 1.5)
            else:
                self.move_cmd.angular.z = max(self.move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

        (position, rotation) = self.get_odom()
        rotation_init = rotation

        while abs((rotation-rotation_init) - goal_z) > 0.05:
            (position, rotation) = self.get_odom()
            if 0 < goal_z < pi:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = 0.5
            else:
                self.move_cmd.linear.x = 0.00
                self.move_cmd.angular.z = -0.5
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()

        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def findBall(self, go_center, width, distance):
        # print('width is: ', width)
        # print('distance is: ', distance)
        (position, rotation) = self.get_odom()
        self.check_car_bound(position, rotation)
        print('center: ', go_center[0], 'position.x: ', position.x, 'position.y: ', position.y, 'rotation: ', rotation)
        # need to GOAL
        # if self.DEBUG:
        #     self.move_cmd.angular.z = 0.0
        #     self.move_cmd.linear.x = 1.2
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.r.sleep()
        if self.FIRST:
            print('first~~~')
            (position, rotation) = self.get_odom()
            # self.move_cmd.angular.z = 3.14
            # self.cmd_vel.publish(self.move_cmd)
            # self.r.sleep()
            
            # print('rotation.x: ', rotation.x, 'rotation.y: ', rotation.y)
            goal_rotation = rotation + pi/12.0
            goal_rotation = self.check_bd(goal_rotation)
            while abs(rotation - goal_rotation) > 0.05:
                self.move_cmd.angular.z = 0.6
                self.move_cmd.linear.x = 0.0
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
                (position, rotation) = self.get_odom()
            while position.x < 1.0: # 1.7
                print('x: ', position.x, ' y: ',position.y) 
                self.move_cmd.angular.z = 0.0
                self.move_cmd.linear.x = 1.2
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
                (position, rotation) = self.get_odom()
            goal_rotation = rotation - pi/12.0# pi/12.0
            goal_rotation = self.check_bd(goal_rotation)
            while abs(rotation - goal_rotation) > 0.05:
                self.move_cmd.angular.z = -0.6
                self.move_cmd.linear.x = 0.0
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
                (position, rotation) = self.get_odom()
            # goal
            # while position.x < 1.5:# 1.3
            #     print('x: ', position.x, 'y: ', position.y)
            #     self.move_cmd.angular.z = 0.0
            #     self.move_cmd.linear.x = 1.2
            #     self.cmd_vel.publish(self.move_cmd)
            #     self.r.sleep()
            #     (position, rotation) = self.get_odom()
            # while position.x > 1.1:
            #     print('x: ', position.x, 'y: ', position.y)
            #     self.move_cmd.angular.z = 0.0
            #     self.move_cmd.linear.x = -1.2
            #     self.cmd_vel.publish(self.move_cmd)
            #     self.r.sleep()
            #     (position, rotation) = self.get_odom()
            
            self.FIRST = False
        elif self.GOAL:
            print('Goaling ~~~')
            self.move_cmd.angular.z = 0.0
            self.move_cmd.linear.x = 1.5
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            # if lose the ball... 
            # if go_center[0] == 0:
            #     self.counter += 1
            # else:
            #     self.counter = 0
            # if self.counter == 3:
            #     print('counter is 3 ======')
            #     self.GOAL = False
            if go_center[0] == 0:
                self.GOAL = False
        
        # close to ball and turn around
        elif width >= 300.0: # and 280 < go_center[0] < 360:  ljx
            # self.move_cmd.angular.z = 0.0
            # self.move_cmd.linear.x = 0.0
            # self.cmd_vel.publish(self.move_cmd)
            # time.sleep(20)
            print('turning~~') 
            alpha = self.get_alpha()
            # print('alpha is ', alpha)
            (position, rotation) = self.get_odom()
            if position.x >= 2.0:
                self.GOAL = True
            elif alpha > pi/6.0:
                direction = None
                #(position, rotation) = self.get_odom()
                if ( abs(rotation) < pi/2.0 and position.y > (self.ballDoor_enemy[1]/100.0-0.20) ) or (abs(rotation) < pi and abs(rotation) > pi/2.0 and position.y < (self.ballDoor_enemy[1]/100.0-0.20) ):
                    print('rotation: ', rotation,' position.x: ', position.x ,' posisition,y: ', position.y, ' door: ', self.ballDoor_enemy[1]/100.0 )
                    direction = 0
                    alpha *= -1
                else:
                    direction = 1
                print('#############################################    direction: ', direction)
                self.turn_around(angle = alpha, direction = direction, distance=distance)# direction, 1(anti-clock-wise)
                self.GOAL=True
            else:
                self.GOAL=True

        # no ball visible
        elif go_center[0] == 0:
            print("!!!!!!!!!!!!!!", go_center[0])
            self.move_cmd.angular.z = 0.8
            self.move_cmd.linear.x = 0.0
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
        elif go_center[0] < 280: # 0 < go_center[0] < 280:
            print("@@@@@@@@@@@@@@@", go_center[0])
            self.move_cmd.angular.z = +0.1
            self.move_cmd.linear.x = 0.0
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
        elif 360 < go_center[0] : # < 640:
            print("###############", go_center[0])
            self.move_cmd.angular.z = -0.1
            self.move_cmd.linear.x = 0.0
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
        else: # 280 <= go_center[0] <= 360:
            # if 310 <= go_center[0] and go_center[0] <= 330:
            print("$$$$$$$$$$$$$$", go_center[0])
            self.move_cmd.angular.z = 0.0
            self.move_cmd.linear.x=0.6 # 0.2
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
           # else:
           #     delta = 0.05 if go_center[0] < 310 else -0.05 
            #    print('clclclclclclclclclcl', go_center[0])
             #   self.move_cmd.angular.z = delta
             #   self.move_cmd.linear.x = 0.0
              #  self.cmd_vel.publish(self.move_cmd)
               # self.r.sleep()
                
            

    def turn_around(self, angle=pi/3.0, direction=1,  distance=100.0):
        # turn right/left for 90 and start circle
        # then turn left/right for 90
        # (position, rotation) = self.get_odom()
        # tmp_rotation = rotation
        # while tmp_rotation
        # for i in range(20):
        #     # (position, rotation) = self.get_odom()
        #     self.move_cmd.angular.z = 0.3
        #     self.move_cmd.linear.x = 0.2
        #     self.cmd_vel.publish(self.move_cmd)
        #     self.r.sleep()
        turn_angular = None
        if direction == 1:
            turn_angular = -0.6
        else:
            turn_angular = 0.6
        (position, rotation) = self.get_odom()
        if direction == 1:
            goal_rotation = rotation - pi/2.0
        else:
            goal_rotation = rotation + pi/2.0

        goal_rotation = self.check_bd(goal_rotation)
        while abs(rotation - goal_rotation) > 0.05:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = turn_angular
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            if( self.check_car_bound(position, rotation) == False):
                return
            (position, rotation) = self.get_odom()

        # turn around
        r = 0.2
        linear_speed = 2*r*pi/4
        angular_speed = 2*pi/4
        if direction == 0:
            angular_speed *= -1
        (position, rotation) = self.get_odom()
        if direction == 1:
            goal_rotation = rotation + angle #suppose the get_a return sign              # self.get_alpha()
        else:
            goal_rotation = rotation - angle
        goal_rotation = self.check_bd(goal_rotation)
        while abs(rotation - goal_rotation) > 0.05:
            try:
                self.move_cmd.linear.x = linear_speed
                self.move_cmd.angular.z = angular_speed
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
                if( self.check_car_bound(position, rotation) == False):
                    return
                (position, rotation)=self.get_odom()
            except:
                rospy.loginfo('shutdown when turning')

        # turn left for 90
        (position, rotation) = self.get_odom()
        if direction == 1:
            goal_rotation = rotation + pi/2.0
        else:
            goal_rotation = rotation - pi/2.0
        goal_rotation = self.check_bd(goal_rotation)
        while abs(rotation - goal_rotation) > 0.05:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = -turn_angular
            self.cmd_vel.publish(self.move_cmd)
            self.r.sleep()
            if( self.check_car_bound(position, rotation) == False):
                return
            (position, rotation) = self.get_odom()
        self.GOAL = True
        # self.goal() # TODO: make it timely

    # keep goal_rotation in (-pi, pi)
    def check_bd(self, goal_rotation):
        if goal_rotation >= pi:
            goal_rotation -= 2*pi
        elif goal_rotation <= -pi:
            goal_rotation += 2*pi
        return goal_rotation

    def get_alpha(self):
        
        # return pi/3.0
        (position, rotation)=self.get_odom()
        me_x, me_y, me_z = position.x, position.y, rotation
        gate_x, gate_y = self.ballDoor_enemy[0]/100.0-0.10, self.ballDoor_enemy[1]/100.0-0.20# 0.35
        a = 0.24
        # notice that I have considered only several situations
        if rotation < 0 and rotation >= -pi/2.0:
            ball_x = me_x + math.cos(abs(me_z))*a
            ball_y = me_y - math.sin(abs(me_z))*a
        elif rotation > -pi and rotation < -pi/2.0:
            ball_x = me_x - math.sin(abs(me_z) - pi/2.0)*a
            ball_y = me_y - math.cos(abs(me_z) - pi/2.0)*a
        elif me_z > 0 and me_z < pi/2.0:
            ball_x = me_x + math.cos(me_z)*a
            ball_y = me_y + math.sin(me_z)*a
        else:
            ball_x = me_x - math.sin(me_z - pi/2.0)*a
            ball_y = me_y + math.cos(me_z - pi/2.0)*a
        b = math.sqrt(pow( abs(me_x - gate_x),2) + pow( abs(me_y - gate_y),2) )
        c = math.sqrt(pow( abs(ball_x - gate_x),2) + pow(abs(ball_y - gate_y),2) )
        
        # b = dis(me, gate)
        # c = dis(bal, gate)
        cos_beta = ( pow(a,2) + pow(c,2) - pow(b,2) ) / (2*a*c)

        #TODO consider about the sign of cos 
        print(cos_beta)
        if cos_beta > 1.0:
            cos_beta = 1.0
        elif cos_beta < -1.0:
            cos_beta = -1.0
        alpha = pi - math.acos( cos_beta )
        print('alpha is: ', alpha)
        return alpha

    # kick the ball
    def goal(self):
        while not rospy.is_shutdown():
            try:
                self.move_cmd.linear.x = 0.5
                self.move_cmd.angular.z = 0.0
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
            except:
                rospy.loginfo('shutdown when goaling')
        # goto gate and detect if loss the ball, if loss , go back directly
    def check_car_bound(self, position, rotation):
        if ( position.y <= -0.2 and rotation < 0.0)or ( position.y >= 1.3 and rotation > 0.0 ) or position.x >= 2.3: # position.y = 1.2
            rr = 3
            if position.x >= 2.3:
                rr += 38
                self.GOAL = False
            for i in range(rr):
                self.move_cmd.linear.x = -0.2
                self.move_cmd.angular.y = 0.0
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
            return False
        elif position.x > 1.5 and self.BACK == True:
            while position.x < 1.3:
                self.move_cmd.angular.z = 0.0
                self.move_cmd.linear.x = -1.2
                self.cmd_vel.publish(self.move_cmd)
                self.r.sleep()
                (position, rotation) = self.get_odom()
            self.BACK = False
        
        
        return True
def main():
    turtlebotNavigation = TurtlebotNavigation()
    res = RES()
    res.listener()
    
    a = input('start')
    while not rospy.is_shutdown():
        #键盘输入，定点程序
        # x, y, z = raw_input("| x | y | z |\n").split()
        # x, y, z = [float(x), float(y), float(z)] 
        # turtlebotNavigation.penalty(x, y, z)

        #opencv检测结果输出
        get_label, get_left, get_top, get_right, get_bottom, get_center = res.get_img_res()
        width = get_right - get_left
        height = get_bottom - get_top
        x = max(height, width)
        distance = 1.237e-12*pow(x,6) -2.225e-9 *pow(x,5) + 
                1.576e-6*pow(x,4) -0.0005623*pow(x,3) + 0.1071*pow(x,2) -10.65*x +498.3
        for i in get_center:
            turtlebotNavigation.findBall(get_center, width , distance)

if __name__ == '__main__':
    main()
