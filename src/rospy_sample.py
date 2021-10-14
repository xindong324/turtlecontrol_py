# coding=UTF-8

#
#ros python示例程序，待添加状态机
import datetime
import math
import time
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from tf.transformations import euler_from_quaternion
import threading

# def rospy
rospy.init_node('logic_main', anonymous=True)

##********************* finite state machine*****************************#
#state def
# state enum
state = {'rotate_to_find_ball': 0, 'follow_and_stop': 1, 'wait_ball_to_stable': 2, 'rotate_self': 3,
         'rotate_around_the_ball': 4, 'rotate_self_to_ball': 5, 'follow_and_push_with_back': 6,'rotate_until_zero':7}
# cur state value
state_value = 0



##### callback func
def call_back():
    global rotate,state_value
    #_,rotate=get_odom()
    # some important state check
    if 0:
        if 0:
            state_value=state['rotate_until_zero']
        if 0:
            state_value=state['follow_and_push_with_back']
    state_to_call[state_value]()


### callback func of each func
def rotate_to_find_ball():
    # global state_value is vital for each callback func without which fsm cannot work
    global state_value
    rospy.loginfo("rotate_to_find_ball!!!!  z=0.5  ")
    ##some useful code
    # calculate
    # pub msg
    #state transfer condition
    if(1):
        state_value = state["follow_and_stop"]
    elif(0):
        state_value = state_value["wait_ball_to_stop"]

def follow_and_stop():
    global state_value
    rospy.loginfo("follow_and_stop!!!!    ")
    if 1:
        state_value = state['wait_ball_to_stable']

    if 0:
        state_value = state['rotate_to_find_ball']


def wait_ball_to_stable():
    global state_value
    rospy.loginfo("wait_ball_to_stable!!!! ")
    pass

def rotate_self():
    global state_value
    pass

def rotate_around_the_ball():
    global state_value
    pass

def rotate_self_to_ball():
    global state_value
    pass

def rotate_until_zero():
    global state_value
    pass

def follow_and_push_with_back():
    global state_value
    pass

######### state func to callback
state_to_call = [rotate_to_find_ball, follow_and_stop, wait_ball_to_stable, rotate_self, rotate_around_the_ball,
                 rotate_self_to_ball, follow_and_push_with_back,rotate_until_zero]


#how to def a new state
# 1. add a new state and its id in "state" array like: 'fly':8
# 2. add a new state callback func. template:
'''
def fly():
    global state_value
    # some functional code
    # process your data and pub sth
    
    #define your state transfer condition 
    if 1:
        state_value = state['wait_ball_to_stable']

    if 0:
        state_value = state['rotate_to_find_ball']
    # try to refer your state by name other than number to avoid confused
'''
# 3. add your callback func name in state_to_call array

####***************************end fsm***********************************#

#*********************************************************************#
##### private func

def thread_job():
    rospy.spin()

# 四元数转欧拉角
# param: msg: geometry_msgs::PoseStamped.Pose
# return: roll, pitch, yaw
def quat2eular(msg):
    (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    return r,p,y

#计算经纬度，输入为图片内目标中心像素点坐标
def point2gps(dis_x,dis_y):
    #print("p2gps")
    pass
###****************end private func***********************************#

#### ros callback func
def posecallback(data):
    #print("pose")
    pass

def Gps(data):
    #print("gps")
    pass
######### end ros callback func
   

def talker():
        #spin 用于回调函数的单独线程
    add_thread = threading.Thread(target = thread_job) 
    
    rospy.Subscriber("/mavros/global_position/global",NavSatFix,Gps)
    rospy.Subscriber("mavros/local_position/pose",PoseStamped,posecallback)
    #启动各个回调函数
    add_thread.start()
    rospy.sleep(1)
    rate = rospy.Rate(20)

    #执行除回调外的其他函数
    while(1):
        call_back()
        rate.sleep()
    #程序结束删除回调线程
    add_thread.stop()


#### main func
try:
    talker()
except rospy.ROSInterruptException:
    pass
