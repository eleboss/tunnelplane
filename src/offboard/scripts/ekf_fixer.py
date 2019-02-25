#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

import rospy
import time
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

InitVins = True
Received_odom = False



def callback_vins(vins):
    global InitVins
    if InitVins:
        InitVins = False


def callback_odom(odom):
    global Received_odom
    Received_odom = True



rospy.init_node('ekf_fixer')

subvins = rospy.Subscriber('vins_estimator/odometry', Odometry, callback_vins)
pub_pose = rospy.Publisher('mavros/vision_pose/pose',PoseStamped, queue_size=1)
subodom = rospy.Subscriber('mavros/local_position/odom', Odometry, callback_odom)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    # 如果没有开始接收ins，那么先用虚拟帧触发EKF
    if InitVins and not Received_odom:
        UAVPose = PoseStamped()
        UAVPose.header.stamp = rospy.Time.now()
        UAVPose.pose.position.x = 0.0
        UAVPose.pose.position.y = 0.0
        UAVPose.pose.position.z = 0.0
        pub_pose.publish(UAVPose)

    rate.sleep()
