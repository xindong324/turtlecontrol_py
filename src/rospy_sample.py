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


def thread_job():
    rospy.spin()

def quat2eular(msg):
    (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    return r,p,y

def posecallback(data):
    print("pose")

def Gps(data):
    print("gps")


#计算经纬度，输入为图片内目标中心像素点坐标
def point2gps(dis_x,dis_y):
    print("p2gps")
   

def talker():
    rospy.init_node('point2gps', anonymous=True)
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
        point2gps(640,720)
        rate.sleep()
    #程序结束删除回调线程
    add_thread.stop()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

