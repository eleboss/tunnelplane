#!/usr/bin/python2.7
# coding=<encoding name> 例如，可添加# coding=utf-8
#主流程控制器

import rospy 
import roslib
import PID
import time
import tf
import RT
import numpy as np
from apriltags.msg import AprilTagDetections
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import PositionTarget, AttitudeTarget, State, RCIn
from geometry_msgs.msg import TransformStamped, PoseStamped, Point, PointStamped, Vector3, Vector3Stamped, TwistStamped, QuaternionStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from offboard.msg import DroneInfo

setpoint_yaw = 0
setpoint_x = setpoint_y = setpoint_z = 0
enable_setpoint = 1
KNOB_L = KNOB_R = BACK_ADJ = 100
SWITCH = 100
detected_tags = 0
feedback_mode = 0
tko_x = tko_y = 0
tko_z = 0.25

pos_fuse_x = pos_fuse_y = pos_fuse_z = 0
odom_roll = odom_pitch = odom_yaw = 0
InitTko = True
search_direction = 1
TAKEOFF_ENABLE = True

OUT = False

TAG_TkO_X = -2.56
TAG_TkO_Y = -0.08

search_x = 1
search_y = 0

def callback_rc(rc):
    global SWITCH, KNOB_L, KNOB_R, BACK_ADJ

    if rc.channels[8] == 1065:
        SWITCH = 1 #向上
    if rc.channels[8] == 1933:
        SWITCH = 0 #向下
    if rc.channels[5] > 1065 and rc.channels[5] < 1355:
        KNOB_L = 0
    if rc.channels[5] >= 1355 and rc.channels[5] < 1644:
        KNOB_L = 1
    if rc.channels[5] >= 1644 and rc.channels[5] < 1933:
        KNOB_L = 2

    if rc.channels[7] > 1065 and rc.channels[7] < 1355:
        KNOB_R = 0
    if rc.channels[7] >= 1355 and rc.channels[7] < 1644:
        KNOB_R = 1
    if rc.channels[7] >= 1644 and rc.channels[7] < 1933:
        KNOB_R = 2

    if rc.channels[6] == 1065:
        BACK_ADJ = 1
    else:
        BACK_ADJ = 0
    # print 'SWITCH',SWITCH,'KNOB_L', KNOB_L,'KNOB_R', KNOB_R, 'BACK_ADJ',BACK_ADJ


def callback_tag(tag):
    global detected_tags
    detected_tags = np.shape(tag.detections)[0]

def callback_odom(odom):
    global pos_fuse_x, pos_fuse_y,pos_fuse_z, odom_roll,odom_pitch,odom_yaw, tko_x, tko_y, InitTko

    pos_fuse_x = odom.pose.pose.position.x
    pos_fuse_y = odom.pose.pose.position.y
    pos_fuse_z = odom.pose.pose.position.z
    #开机的时候，开关是在降落状态的
    if SWITCH == 1 and InitTko:
        tko_x = pos_fuse_x
        tko_y = pos_fuse_y
        InitTko = False
    qn_odom = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    (odom_roll,odom_pitch,odom_yaw) = euler_from_quaternion(qn_odom)
    

rospy.init_node('drone_info')
rc_in = rospy.Subscriber('mavros/rc/in', RCIn, callback_rc, queue_size=1)
position_tag = rospy.Subscriber('apriltags/detections', AprilTagDetections, callback_tag, queue_size=1)
subodom = rospy.Subscriber('mavros/local_position/odom', Odometry, callback_odom, queue_size=1)

pub_droneinfo =  rospy.Publisher('drone_info', DroneInfo, queue_size=1)

rate = rospy.Rate(100)
while not rospy.is_shutdown():
    dinfo = DroneInfo()

    #Takeoff
    if SWITCH == 0:
        if (pos_fuse_z < tko_z):
            if TAKEOFF_ENABLE: 
                setpoint_x = tko_x
                setpoint_y = tko_y
                setpoint_z = tko_z
                TAKEOFF_ENABLE = False

    if KNOB_R == 2:
        feedback_mode = 0
        if setpoint_x >= search_x:
            pass
        else:
            setpoint_x = setpoint_x + 0.0009
        setpoint_y = tko_y
        setpoint_z = tko_z

    elif KNOB_R == 1 :
        if OUT:
            feedback_mode = 0
            # setpoint_x = pos_fuse_x
            # setpoint_y = pos_fuse_y  
            setpoint_z = tko_z
            OUT = False
    elif KNOB_R == 0 :
        feedback_mode = 0
        setpoint_x = setpoint_x - 0.0006
        setpoint_z = tko_z


    if KNOB_L == 2:
        pass
    elif KNOB_L == 1:
        pass
    elif KNOB_L == 0:
        pass


    if BACK_ADJ == 1:
        pass
    else:
        pass

    # LANDING
    if SWITCH == 1:
        setpoint_z = 0
                   
    dinfo.header.stamp = rospy.get_rostime()
    dinfo.setpoint.position.x = setpoint_x
    dinfo.setpoint.position.y = setpoint_y
    dinfo.setpoint.position.z = setpoint_z
    dinfo.setpoint.orientation.w = setpoint_yaw
    dinfo.enable_setpoint = enable_setpoint
    dinfo.fmode = feedback_mode #VINS or TAG
    pub_droneinfo.publish(dinfo)


    print 'setpoint',setpoint_x,setpoint_y,setpoint_z

    rate.sleep()
