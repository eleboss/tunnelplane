#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8

import rospy 
import roslib
import PID
import time
import tf
import numpy as np
from apriltags.msg import AprilTagDetections
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State, RCIn
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped, Vector3, Vector3Stamped, TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion


rospy.init_node('delay')
last_range = last_time = 0

def callback_range(range):
    global last_range, last_time

    if range.range - last_range > 0.4:
        print 'current time', time.time(), 'lasttime', last_time, 'dtime',time.time() - last_time

    last_range = range.range
    last_time = time.time()

subrange = rospy.Subscriber('/mavros/distance_sensor/tfmini_pub', Range, callback_range)

#rospy.spin()


rate = rospy.Rate(100)  
while not rospy.is_shutdown():



    stamp = rospy.get_rostime()
    # print time.time()



    rate.sleep()